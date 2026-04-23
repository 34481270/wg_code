import sys
project_root = r"C:\Users\david ho\Desktop\WG"
if project_root not in sys.path:
    sys.path.insert(0, project_root)
import multiprocessing as mp
try:
    mp.set_start_method('spawn', force=True)
except RuntimeError:
    pass
from geopy.distance import geodesic
import numpy as np
import time
import math
import os
import csv
from queue import Empty
import threading
import serial
from scipy.integrate import solve_ivp
from WG.sensor import GPS_IMU_MOCK as GPS_IMU  # type: ignore
from WG.sensor import gps_utils, socket_client  # type: ignore
# from WG.sensor import wind  # type: ignore
from WG.Model import findHeading, findVirtualTarget, wavegliderDynamic, waveGenerator, KalmanEstimateCurrent, PID # type: ignore
from enum import Enum, auto

class Cmd(Enum):
    START = auto()
    STOP = auto()
    SHUTDOWN = auto()
    RUDDER_TEST_START = auto()
    RUDDER_TEST_STOP = auto()
    PROPELLER_TEST_START = auto()
    PROPELLER_TEST_STOP = auto()
    
# 2) command_router：統一在這裡改共享旗標
def command_router(cmd_q, shutdown_evt, shared, sender):
    while not shutdown_evt.is_set():
        try:
            cmd = cmd_q.get(timeout=0.2)
        except Exception:
            continue

        if cmd == Cmd.START:
            shared['nav_enabled'].value = 1
            shared['prop_enabled'].value = 1
            sender.sendQueue.put(('State', '開始導航'))

        elif cmd == Cmd.STOP:
            shared['nav_enabled'].value = 0
            shared['prop_enabled'].value = 0
            shared['rudder_test_enabled'].value = 0
            shared['propellerParameter'].value = 0.0
            shared['indicator']['navigation'] = 0
            shared['indicator']['propeller'] = 0
            shared['rudder_angle'].value = 0
            sender.sendQueue.put(('State', '停止導航'))

        elif cmd == Cmd.RUDDER_TEST_START:
            shared['rudder_test_enabled'].value = 1
            shared['prop_enabled'].value = 1
            sender.sendQueue.put(('State', '舵機測試開始'))

        elif cmd == Cmd.RUDDER_TEST_STOP:
            shared['rudder_test_enabled'].value = 0
            shared['prop_enabled'].value = 0
            sender.sendQueue.put(('State', '舵機測試停止'))

        elif cmd == Cmd.PROPELLER_TEST_START:
            shared['prop_enabled'].value = 1
            shared['rudder_test_enabled'].value = 0
            shared['rudder_angle'].value = 0
            sender.sendQueue.put(('State', '螺旋槳測試開始'))

        elif cmd == Cmd.PROPELLER_TEST_STOP:
            shared['prop_enabled'].value = 0
            shared['propellerParameter'].value = 0.0
            shared['indicator']['propeller'] = 0
            sender.sendQueue.put(('State', '螺旋槳測試停止'))

        elif cmd == Cmd.SHUTDOWN:
            shutdown_evt.set()
            shared['nav_enabled'].value = 0
            shared['prop_enabled'].value = 0
            shared['rudder_test_enabled'].value = 0
            shared['propellerParameter'].value = 0.0
            sender.sendQueue.put(('State', '系統關閉'))


def rudderPidTest(shared, shutdown_evt):
    """
    - 在這裡本地 new 一個 PID
    - 參數透過 shared['rudder_kp/ki/kd'] + shared['rudder_pid_ver'] 熱更新
    - 重置透過 shared['shared_rudder_reset_ver']
    - 是否執行測試：看 shared['rudder_test_enabled']
    - 全系統關閉：看 shutdown_evt
    """
    # 本地 PID 實例（初始取共享參數）
    pid = PID.PID(
        Kp=shared['rudder_kp'].value,
        Ki=shared['rudder_ki'].value,
        Kd=shared['rudder_kd'].value
    )

    last_ver = shared['rudder_pid_ver'].value
    last_reset_ver = shared['shared_rudder_reset_ver'].value
    last_rudder_update_time = time.time()

    while not shutdown_evt.is_set():
        # 沒啟動舵機測試 → idle
        # print('rudder_test_enabled : ', shared['rudder_test_enabled'].value)
        if shared.get('rudder_test_enabled', None) and shared['rudder_test_enabled'].value == 0:
            time.sleep(0.05)
            continue

        ver = shared['rudder_pid_ver'].value
        if ver != last_ver:
            pid.rudder_pid_parameter_adjust(
                shared['rudder_kp'].value,
                shared['rudder_ki'].value,
                shared['rudder_kd'].value
            )
            last_ver = ver
        # 參數熱更新（所有 process/thread 的 PID 物件都靠這個同步）

        # 重置（清 integral、微分等）
        reset_ver = shared['shared_rudder_reset_ver'].value
        if reset_ver != last_reset_ver:
            try:
                # 你的 PID 有 setZero 就用；沒有就自行清空
                pid.setZero()
            except AttributeError:
                pid.error_integral = 0.0
                pid.error_prev = 0.0
            last_reset_ver = reset_ver

        now = time.time()
        if now - last_rudder_update_time >= shared['rudder_update_interval'].value:
            # 計算 dt_real
            dt_real = now - last_rudder_update_time
            shared['dt_real'].value = dt_real
            last_rudder_update_time = now

            # 指令與當前航向
            cmd = shared['pidTestCmd'].value
            course_angle = shared['GPS'].get(
                'heading',
                (90 - shared['IMU'].get('angle_yaw', 0.0)) % 360
            )

            # 計算舵角
            rudder_angle = pid.compute(course_angle, cmd, dt_real)
            shared['rudder_angle'].value = rudder_angle

            # 回報到前端（可選）
            shared['simu_sender_queue'].put(("course_angle", course_angle))

        time.sleep(0.01)
        
def bearing(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    theta = math.atan2(x, y)
    # print("期望航向：")
    # print((math.degrees(theta) + 360) % 360)
    return (math.degrees(theta) + 360) % 360

def navigationProcess(shared, shutdown_evt):
    last_ver = -1
    last_reset_ver = -1
    rudder_pid = PID.PID(
        Kp=shared['rudder_kp'].value,
        Ki=shared['rudder_ki'].value,
        Kd=shared['rudder_kd'].value
    )
    
    arrived = False
    
    while not shutdown_evt.is_set():
        if arrived:
            try:
                gps = dict(shared['GPS'])
                goal = list(shared['goal'])
                dis2goal = geodesic([gps.get('lat', None), gps.get('lon', None)], [goal[1], goal[0]]).meters
                if dis2goal > shared['back_to_goal_dis'].value:
                    pathPlanningProcess(shared)
                    shared['simu_sender_queue'].put(("State", '飄遠重新啟動'))
                    shared['prop_enabled'].value = 1
                    shared['nav_enabled'].value = 1
                    arrived = False
            except Exception:
                pass
        
        # 沒開導航就 idle
        if shared['nav_enabled'].value == 0:
            time.sleep(0.05)
            continue
        


        last_rudder_update_time = time.time()
        flow_blocked = False

        # === 導航主迴圈（直到被關或停導航） ===
        while (not shutdown_evt.is_set()) and (shared['nav_enabled'].value == 1):
            # ---- Rudder PID 參數熱更新 ----
            ver = shared['rudder_pid_ver'].value
            if ver != last_ver:
                rudder_pid.rudder_pid_parameter_adjust(
                    shared['rudder_kp'].value,
                    shared['rudder_ki'].value,
                    shared['rudder_kd'].value
                )
                last_ver = ver

            reset_ver = shared['shared_rudder_reset_ver'].value
            if reset_ver != last_reset_ver:
                rudder_pid.setZero()
                last_reset_ver = reset_ver

            # 狀態更新
            shared['indicator']['navigation'] = 1
            goal = list(shared['goal'])
            now = time.time()

            gps = dict(shared['GPS'])
            lat = gps.get('lat', 0.0)
            lon = gps.get('lon', 0.0)
            shared['gps_history_kalman'].append([lat, lon])

            if now - last_rudder_update_time >= shared['rudder_update_interval'].value:
                # === Kalman 區段（維持你的原始邏輯） ===
                print('simu_counter ',shared['simu_counter'].value)
                print('reset_simulation ',shared['reset_simulation'].value)
                if shared['reset_simulation'].value:
                    simulated_path = list(shared['simulated_path'])

                    simu_lat = shared['simu_gps']['lat']
                    simu_lon = shared['simu_gps']['lon']
                    x0, y0 = gps_utils.gps_to_utm(simu_lat, simu_lon)

                    simu_gps_path = []
                    for x_rel, y_rel in simulated_path[:]:
                        x = x0 + x_rel
                        y = y0 + y_rel
                        lat_, lon_ = gps_utils.utm_to_gps(x, y)
                        simu_gps_path.append([lat_, lon_])

                    shared['simu_sender_queue'].put(("drawSimulate", simu_gps_path))

                    gps_history = list(shared['gps_history_kalman'])
                    utm_history = []
                    for lat_, lon_ in gps_history:
                        x_, y_ = gps_utils.gps_to_utm(lat_, lon_)
                        utm_history.append([x_, y_])

                    Uc_est, Vc_est, P_new, V = KalmanEstimateCurrent.KalmanEstimateCurrent(
                        np.array(utm_history),
                        np.array(simu_gps_path),
                        shared['kalman_interval'].value,
                        shared['Uc'].value,
                        shared['Vc'].value,
                        np.array(shared['P'])
                    )

                    shared['Uc'].value = 0#Uc_est
                    shared['Vc'].value = 0#Vc_est
                    shared['P'][:] = P_new.tolist()
                    shared['current_speed'].value = np.hypot(shared['Uc'].value, shared['Vc'].value)
                    shared['current_angle'].value = np.degrees(np.arctan2(shared['Vc'].value, shared['Uc'].value))

                    # reset
                    shared['gps_history_kalman'][:] = []
                    shared['simulated_state'][:] = [
                        0.0, 0.0, 0.0, shared['IMU'].get('angle_yaw', 0.0),
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 9.5, shared['IMU_Glider'].get('angle_yaw', 0.0),
                        0.0, 0.0, 0.0, 0.0
                    ]
                    shared['simulated_path'][:] = []
                    shared['simu_counter'].value = 0
                    shared['reset_simulation'].value = 0
                    shared['simu_gps']['lat'] = gps.get('lat', 0.0)
                    shared['simu_gps']['lon'] = gps.get('lon', 0.0)

                # === 計算期望航向 ===
                current_path = list(shared['current_path'])
                dubins_path  = list(shared['dubins_path'])
                min_dist, heading_dubins = None, None
                if current_path and dubins_path and gps.get('lat') is not None:
                    try:
                        min_dist, heading_dubins = findHeading.findHeading(
                            np.array(current_path),
                            [gps['lat'], gps['lon']],
                            np.array(dubins_path)
                        )
                    except Exception as e:
                        print(f"findHeading 例外: {e}")
                        last_rudder_update_time = now
                        time.sleep(0.01)
                        continue

                # allow_flow_block = (
                #     (min_dist is not None) and 
                #     (min_dist >= shared['replan_dis'].value)
                # )

                # current_speed = shared['current_speed'].value
                # wg_speed      = shared['wg_speed'].value

                # if not flow_blocked:
                #     if allow_flow_block and (current_speed > wg_speed):
                #         flow_blocked = True
                #         shared['simu_sender_queue'].put(("State", '流速過大'))
                # else:
                #     if current_speed <= wg_speed * 1.05:
                #         flow_blocked = False
                #         shared['simu_sender_queue'].put(("State", '流速緩和'))
                #         pathPlanningProcess(shared)
                #         shared['simu_sender_queue'].put(("drawTrajectory", list(shared['current_path'])))
                #         shared['last_path_sent_version'].value += 1

                gps_speed   = shared['GPS'].get('speed', 0.0)
                gps_heading = shared['GPS'].get('heading', None)
                if gps_heading is not None and gps_speed > 0.2:
                    course_angle = gps_heading
                else:
                    course_angle = (90 - shared['IMU'].get('angle_yaw', 0.0)) % 360

                if flow_blocked:
                    if goal:
                        goal_lon, goal_lat = goal  # [lon, lat]
                        desired_heading = bearing(gps['lat'], gps['lon'], goal_lat, goal_lon)
                        shared['exceptCourse'].value = desired_heading
                        rudder_angle = rudder_pid.compute(course_angle, desired_heading, shared['dt_real'].value)
                        shared['rudder_angle'].value = rudder_angle
                else:
                    if heading_dubins is not None:
                        shared['exceptCourse'].value = heading_dubins
                        rudder_angle = rudder_pid.compute(course_angle, heading_dubins, shared['dt_real'].value)
                        shared['rudder_angle'].value = rudder_angle

                # 模擬（保留）
                modelSimulationProcess(shared)

                # 更新 dt_real
                dt_real = now - last_rudder_update_time
                shared['dt_real'].value = dt_real
                last_rudder_update_time = now

                # 抵達目標：關閉導航與螺旋槳（讓各自 process/thread 自己收尾）
                try:
                    dis2goal = geodesic([gps.get('lat', 0.0), gps.get('lon', 0.0)], [goal[1], goal[0]]).meters
                    if dis2goal < shared['success_threshold'].value:
                        shared['simu_sender_queue'].put(("State", '已到達目標'))
                        shared['indicator']['navigation'] = 0
                        shared['indicator']['propeller'] = 0
                        # shared['nav_enabled'].value = 0
                        shared['prop_enabled'].value = 0
                        # 如需清積分，順手要求一次重置：
                        shared['prop_reset_ver'].value += 1
                        shared['shared_rudder_reset_ver'].value += 1
                        shared['rudder_angle'].value = 0
                        shared['nav_enabled'].value = 0
                        arrived = True
                        break  # 回到外層等待
                except Exception:
                    pass   
                
                # 偏差過大 → 重規劃
                if (min_dist is not None) and (min_dist >= shared['replan_dis'].value or list(shared['goal']) != goal):
                    shared['simu_sender_queue'].put(("State", '誤差過大重新規劃'))
                    pathPlanningProcess(shared)
                    
                    
            

            time.sleep(0.01)

        # idle 小睡
        time.sleep(0.05)

        
def pathPlanningProcess(shared): 
    # ===  等待 GPS 初始化 === #
    gps = shared['GPS']
    while True:
        start_lat = gps.get('lat', None)
        start_lon = gps.get('lon', None)
        if start_lat is not None and start_lon is not None:
            break
        print(" 等待 GPS 資料中")
        time.sleep(0.5)

    goal = list(shared['goal'])  # [lat, lon]
    if len(goal) < 2:
        return


    # 把 goal 轉為相對座標
    
    x, y = gps_utils.gps_to_relative_xy(start_lat, start_lon, goal[1], goal[0])
    theta = np.arctan2(y, x)

    # 找相對路徑


    h = gps.get('heading', None)
    if h is not None:
        heading = (90 - h) % 360
    else:
        heading = shared['IMU'].get('angle_yaw', 0.0)
    print('heading :',heading)
    dubins_path, current_path = findVirtualTarget.findVirtualTarget(
        np.array([0, 0, np.radians(heading)]),
        np.array([x, y, theta]),
        shared['R'].value,
        shared['current_speed'].value,
        shared['wg_speed'].value,
        shared['current_angle'].value,
    )
    # 轉回絕對 UTM 再轉 GPS 經緯度
    x0, y0 = gps_utils.gps_to_utm(start_lat, start_lon)
    dubins_path_xy = dubins_path + np.array([x0, y0])
    
    x1, y1 = gps_utils.gps_to_utm(start_lat, start_lon)
    current_path_path_xy = current_path + np.array([x1, y1])

    dubins_path_latlon = [
        list(gps_utils.utm_to_gps(x, y))  # 轉成 [lat, lon]
        for x, y in dubins_path_xy
    ]
    
    current_path_latlon = [
        list(gps_utils.utm_to_gps(x, y))  # 轉成 [lat, lon]
        for x, y in current_path_path_xy
    ]
    
    shared['dubins_path'][:] = dubins_path_latlon
    shared['current_path'][:] = current_path_latlon
    
    shared['current_path_history'].append(current_path_latlon)
    shared['dubins_path_history'].append(dubins_path_latlon)
    shared['path_version'].value += 1
    log_path_planning(shared)
    shared['simu_sender_queue'].put(("State", "路徑規劃完成"))  

    
    
    
def log_path_planning(shared):
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    goal = list(shared['goal'])
    current_path = list(shared['current_path'])
    dubins_path = list(shared['dubins_path'])

    exp_dir = shared['exp_dir']  

    current_file = os.path.join(exp_dir, f"current_path_{timestamp}.xtxt")
    with open(current_file, "w", encoding="utf-8") as f:
        f.write(f"# Timestamp: {timestamp}\n")
        f.write(f"# Goal: {goal}\n")
        f.write("lat,lon\n")
        for lat, lon in current_path:
            f.write(f"{lat:.8f},{lon:.8f}\n")

    dubins_file = os.path.join(exp_dir, f"dubins_path_{timestamp}.txt")
    with open(dubins_file, "w", encoding="utf-8") as f:
        f.write(f"# Timestamp: {timestamp}\n")
        f.write(f"# Goal: {goal}\n")
        f.write("lat,lon\n")
        for lat, lon in dubins_path:
            f.write(f"{lat:.8f},{lon:.8f}\n")               

def modelSimulationProcess(shared):
    rudder_angle = shared['rudder_angle'].value
    dt_real = shared['dt_real'].value 
    current_angle = shared['current_angle'].value
    simu_counter = shared['simu_counter'].value 
    shared['simu_counter'].value += dt_real
    
    t_cal = np.linspace(0, dt_real, 1000)
    tau_wave = waveGenerator.excited_force(U195=10)
    
    simulated_state = list(shared['simulated_state'])
    
    waveglider_func = lambda t, state: wavegliderDynamic.wavegliderDynamic(t, state, rudder_angle, t_cal, tau_wave, current_angle, 0)
    simulated_state2 = solve_ivp(waveglider_func, [simu_counter, simu_counter + dt_real], np.array(simulated_state), method='RK45')
    
    shared['simulated_state'][:] = list(simulated_state2.y[:,-1]) 
    shared['simulated_path'].extend(np.stack((simulated_state2.y[0], simulated_state2.y[1]), axis=-1)) 
    if shared['simu_counter'].value >= shared['kalman_interval'].value:
        shared['reset_simulation'].value = 1
        
    time.sleep(0.01)

def process_queue_data(shared, sender, cmd_queue):
    def setGoal(data):
        try:
            lat = data['lat']; lng = data['lng']
            shared['goal'][:] = [lng, lat]
            print(f"✔ setGoal：lat={lat}, lon={lng}")
        except Exception as e:
            print(f"✘ setGoal 錯誤：{e}")

    def experimentStart(_=None):
        # 可選：先做一次路徑規劃
        print(list(shared['goal']))
        if len(shared['goal']) >= 2:
            print("實驗開始")
            pathPlanningProcess(shared)
            cmd_queue.put(Cmd.START)

    def experimentFinish(_=None):
        clear_shared_for_next_experiment(shared)
        cmd_queue.put(Cmd.STOP)

    def pid_parameter_adjust(data):
        # 保留你原本的 PID 參數調整，不動事件
        if data[0] == 'propeller':
            shared['prop_kp'].value = data[1]
            shared['prop_ki'].value = data[2]
            shared['prop_kd'].value = data[3]
            shared['prop_pid_ver'].value += 1
            shared['target_speed'].value = data[4]
            cmd_queue.put(Cmd.PROPELLER_TEST_START)
            
            print('!!propeller')
            
            
        elif data[0] == 'rudder':
            shared['rudder_kp'].value = data[1]
            shared['rudder_ki'].value = data[2]
            shared['rudder_kd'].value = data[3]
            shared['rudder_pid_ver'].value += 1
            shared['pidTestCmd'].value = data[6]
            cmd_queue.put(Cmd.RUDDER_TEST_START)
            print('!!rudder')
            
        elif data[0] == 'general':
            shared['target_speed'].value = data[4]
            shared['rudder_angle'].value = data[5]
            print('!!general')

    def pidTestFinish(_=None):
        cmd_queue.put(Cmd.RUDDER_TEST_STOP)
        cmd_queue.put(Cmd.PROPELLER_TEST_STOP)
        shared['shared_rudder_reset_ver'].value += 1
        shared['prop_reset_ver'].value += 1

    handler_map = {
        'setGoal' : setGoal,
        'experimentStart' : experimentStart,
        'experimentFinish' : experimentFinish,
        'UI_is_ready' : lambda _ : sender.UI_is_ready(),
        'pid' : pid_parameter_adjust,
        'pidTestFinish': pidTestFinish,
    }

    while True:
        try:
            item = sender.recvQueue.get(timeout=1)
            if isinstance(item, tuple) and len(item) == 2:
                cmd, data = item
                handler = handler_map.get(cmd)
                if handler: handler(data)
                else: print(f"⚠️ 未知指令：{cmd}")
        except Empty:
            continue
        
def bridgeSimuQueueToSender(shared, sender):
    fieldnames = [
        'log_time','State','wgSpeed','flowSpeed',
        'flowDir','heading','exceptCourse','rudderAngle','windSpeed','depth'
    ]

    current_dir = None
    f = None
    writer = None
    queue = shared['simu_sender_queue']

    while True:
        exp_dir = shared['exp_dir']
        if exp_dir != current_dir:
            if f:
                f.close()
            os.makedirs(exp_dir, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            log_path = os.path.join(exp_dir, f"history_{ts}.csv")
            f = open(log_path, 'w', newline='', encoding='utf-8')
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            current_dir = exp_dir

        try:
            cmd, data = queue.get(timeout=1)
        except Empty:
            continue

        sender.sendQueue.put((cmd, data))

        row = {col: '-' for col in fieldnames}
        row['log_time'] = time.strftime("%Y-%m-%d %H:%M:%S")

        if cmd == 'State':
            row['State'] = data
        elif cmd == 'updateStats':
            for key, val in data.items():
                if key in row:
                    row[key] = val

        writer.writerow(row)
        f.flush()
        
def propeller(shared, shutdown_evt):
    pid = propeller_pid(Kp=shared['prop_kp'].value, Ki=shared['prop_ki'].value, Kd=shared['prop_kd'].value)
    last_ver = shared['prop_pid_ver'].value
    last_reset_ver = shared['prop_reset_ver'].value
    prop_on = False
    
    while not shutdown_evt.is_set():
        # print('target_speed ', shared['target_speed'].value)
        # print('wg_speed ', shared['wg_speed'].value)
        if shared['prop_enabled'].value == 0:
            time.sleep(0.1); continue
        
        # 熱更新
        if shared['prop_pid_ver'].value != last_ver:
            pid.setGains(shared['prop_kp'].value, shared['prop_ki'].value, shared['prop_kd'].value)
            last_ver = shared['prop_pid_ver'].value
        # 重置
        if shared['prop_reset_ver'].value != last_reset_ver:
            pid.setZero()
            last_reset_ver = shared['prop_reset_ver'].value

        # 你的速度控制邏輯（維持原本 now_speed/target_speed 判斷）
        gps = shared['GPS']
        gps_speed = gps.get('speed', 0.0)
        shared['wg_speed'].value = gps_speed

        target = shared['target_speed'].value

        if gps_speed < target - 0.2:
            prop_on = True
        elif gps_speed > target :
            prop_on = False
            
        if prop_on:
            shared['indicator']['propeller'] = 1
            motor_cmd = pid.update(target - gps_speed)
            shared['propellerParameter'].value = motor_cmd
        else:
            shared['indicator']['propeller'] = 0
            shared['propellerParameter'].value = 0
            pid.setZero()

        time.sleep(0.5)

class propeller_pid:
    def __init__(self, Kp, Ki, Kd, output_limits=(0, 100)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output = 0
        
        self.output_limits = output_limits  # (min, max)

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
        self.counter = 0

    def setZero(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
        self.counter = 0

    def setGains(self, Kp, Ki ,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        print(self.Kp)
        print(self.Ki)
        print(self.Kd)
        print('propeller PID參數設定完成')

    def update(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        if dt <= 0.0:
            dt = 1e-16

        P = self.Kp * error
        if abs(self.output) < 100:  
            self.integral += error * dt
        I = self.Ki * self.integral
        if self.counter == 1:
            D = self.Kd * (error - self.prev_error) / dt
        else:
            D = 0
            self.counter = 1

        # è¨ä½ä¸æ¬¡çerror
        self.prev_error = error

        # è¼¸åº
        self.output = P + I + D

        # éå¶è¼¸åºç¯å
        self.output = max(self.output_limits[0], min(self.output, self.output_limits[1]))
        return self.output


class ArduinoManager(threading.Thread):
    def __init__(self, port="/dev/GliderArduino", baudrate=9600, shared=None, retry_interval=2):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.shared = shared
        self.retry_interval = retry_interval
        self.ser = None
        self.cmd_queue = shared['arduino_command_queue']
        self.running = True

    def connect_serial(self):
        while self.running:
            try:
                if self.ser:
                    try:
                        self.ser.close()
                    except:
                        pass
                print(f"Trying to open {self.port} @ {self.baudrate}")
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                print(f"Connected to Arduino on {self.port}")

                self.ser.dtr = False
                time.sleep(0.1)
                self.ser.dtr = True
                time.sleep(2)
                self.ser.reset_input_buffer()
                print("Serial buffer flushed, ready to communicate")
                return

            except serial.SerialException as e:
                print(f"Serial connection failed: {e}. Retrying in {self.retry_interval}s...")
                self.ser = None
                time.sleep(self.retry_interval)

    def run(self):
        while self.running:
            if not self.ser or not self.ser.is_open:
                self.connect_serial()

            try:
                while not self.cmd_queue.empty():
                    cmd = self.cmd_queue.get_nowait()
                    self.ser.write((cmd + "\n").encode("utf-8"))
                    # print(f"Sent to Arduino: {cmd}")
            except Exception as e:
                print(f"Write error: {e}")
                try: self.ser.close()
                except: pass
                self.ser = None
                continue

            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    # print(f"From Arduino: {line}")
                    if '@' in line:
                        cmd, data = line.split('@', 1)
                        self.handle_command(cmd.strip(), data.strip())
            except serial.SerialException as e:
                print(f"? Read error (likely disconnected): {e}")
                try: self.ser.close()
                except: pass
                self.ser = None
                continue
            except Exception as e:
                print(f"Unexpected read error: {e}")

            time.sleep(0.01)

    def handle_command(self, cmd, data):
        if cmd == "pressure":
            try:
                self.shared['pressure'].value = float(data)
            except ValueError:
                print("Invalid pressure data")
        elif cmd == "BatteryVoltage":
            try:
                self.shared['BatteryVoltage'].value = float(data)
            except ValueError:
                pass
        elif cmd == "windSpeed":
            try:
                val = float(data)
                self.shared['windSpeed'].value = val
                print(f"?? windSpeed updated = {val}")
            except ValueError:
                pass
                

class ArduinoReceiver(threading.Thread):
    def __init__(self, port="/dev/FloaterArduino", baudrate=9600, shared=None, retry_interval=2):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.shared = shared
        self.retry_interval = retry_interval
        self.ser = None
        self.running = True

    def connect_serial(self):
        while self.running:
            try:
                if self.ser:
                    try:
                        self.ser.close()
                    except:
                        pass
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                self.ser.dtr = False
                self.ser.dtr = True
                self.ser.reset_input_buffer()
                print("FloaterArduinoé€£ç·æå")
                time.sleep(5)
                return
            except serial.SerialException as e:
                print(f"[Receiver] {e}?{self.retry_interval}s ")
                self.ser = None
                time.sleep(self.retry_interval)

    def run(self):
        while self.running:
            if not self.ser or not self.ser.is_open:
                self.connect_serial()

            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                # print(f"[Receiver] Arduino {line}")

                if '@' in line:
                    cmd, data = line.split('@', 1)
                    cmd = cmd.strip()
                    data = data.strip()
                    self.handle_command(cmd,data)

            except serial.SerialException as e:
                print(f"[Receiver] {e}")
                try:
                    self.ser.close()
                except:
                    pass
                self.ser = None
            except Exception as e:
                print(f"[Receiver] {e}")

            time.sleep(0.01)

    def handle_command(self, cmd, data):
        try:
            if cmd == "BatteryVoltage":
                self.shared['BatteryVoltage'].value = float(data)
            elif cmd == "windSpeed":
                self.shared['windSpeed'].value = float(data)
                print(f"[Receiver] windSpeed {data}")
        except ValueError:
            print(f"[Receiver] {cmd}@{data}")



def clear_shared_for_next_experiment(shared):
    # reset when mission finish
    shared['rudder_angle'].value = 0
    shared['simulated_state'][:] = [0.0, 0.0, 0.0, shared['IMU'].get('angle_yaw', 0.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, shared['IMU_Glider'].get('angle_yaw', 0.0), 0.0, 0.0, 0.0, 0.0]
    shared['simulated_path'][:] = []
    shared['simu_counter'].value = 0
    shared['wg_speed'].value = 0
    shared['now_speed'].value = 0.0
    shared['dubins_path'][:] = []
    shared['current_path'][:] = []
    shared['path_version'].value = 0
    shared['last_path_sent_version'].value = 0
    shared['current_speed'].value = 0
    shared['current_angle'].value = 0
    shared['goal'][:] = []

def myPosition(shared, sender):
    while True:
        gps = shared['GPS']
        lat = gps.get('lat', None) 
        lon = gps.get('lon', None)
        
        if lat is not None and lon is not None:
            sender.sendQueue.put(("addPoint", [lat, lon]))
           
        else:
            print("GPS等待中")
        time.sleep(5)
        
def gpsLogger(shared):
    current_dir = None
    f = None
    last_snapshot = {}
    while True:
        exp_dir = shared['exp_dir']
        if exp_dir != current_dir:
            if f:
                f.close()
            os.makedirs(exp_dir, exist_ok=True)
            # ts = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(exp_dir, "gps.csv")
            f = open(path, 'w', encoding='utf-8', newline='')
            f.write("time,lat,lon,heading,speed\n")  # header
            current_dir = exp_dir
            last_snapshot = {} 

        gps = dict(shared['GPS'])
        if gps.get('lat') is not None and gps != last_snapshot:
            now = time.strftime("%Y-%m-%d %H:%M:%S")
            line = f"{now},{gps.get('lat')},{gps.get('lon')},{gps.get('heading')},{gps.get('speed')}\n"
            f.write(line)
            f.flush()
            last_snapshot = gps

        time.sleep(0.05)

def imuLogger(shared):
    current_dir = None
    f = None
    last_snapshot = {}
    while True:
        exp_dir = shared['exp_dir']
        if exp_dir != current_dir:
            if f:
                f.close()
            os.makedirs(exp_dir, exist_ok=True)
            # ts = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(exp_dir, "imu.csv")
            f = open(path, 'w', encoding='utf-8', newline='')
            f.write("time,angle_yaw,angle_pitch,angle_roll,...\n")
            current_dir = exp_dir
            last_snapshot = {}

        imu = dict(shared['IMU'])
        if imu and imu != last_snapshot:
            now = time.strftime("%Y-%m-%d %H:%M:%S")
            omega = imu.get('angularVelocity_yaw',  '')
            yaw = imu.get('angle_yaw','')
            line = f"{now},{omega},{yaw}\n"
            f.write(line)
            f.flush()
            last_snapshot = imu

        time.sleep(0.01)

def imuGliderLogger(shared):
    current_dir = None
    f = None
    last_snapshot = {}
    while True:
        exp_dir = shared['exp_dir']
        if exp_dir != current_dir:
            if f:
                f.close()
            os.makedirs(exp_dir, exist_ok=True)
            path = os.path.join(exp_dir, "imu_glider.csv")
            f = open(path, 'w', encoding='utf-8', newline='')
            f.write("time,angle_yaw,angle_pitch,angle_roll\n")
            current_dir = exp_dir
            last_snapshot = {}

        gimu = dict(shared['IMU_Glider'])
        if gimu and gimu != last_snapshot:
            now = time.strftime("%Y-%m-%d %H:%M:%S")
            omega = gimu.get('angularVelocity_yaw',  '')
            yaw = gimu.get('angle_yaw','')
            line = f"{now},{omega},{yaw}\n"
            f.write(line)
            f.flush()
            last_snapshot = gimu

        time.sleep(0.01)

def get_next_experiment_folder(log_root="path_logs"):

    os.makedirs(log_root, exist_ok=True)

    existing = [
        d for d in os.listdir(log_root)
        if d.startswith("experiment_") and d.split("_")[-1].isdigit()
    ]
    existing_nums = [int(d.split("_")[-1]) for d in existing] if existing else []
    next_exp_num = max(existing_nums, default=0) + 1

    exp_dir = os.path.join(log_root, f"experiment_{next_exp_num}")
    os.makedirs(exp_dir, exist_ok=True)
    return exp_dir

def generateChecksum(cmd):
    checksum = 0
    for ch in cmd:
        checksum ^= ord(ch)
    return f"{checksum:02X}"

def sensor_thread(shared):
    
    gps_thread = GPS_IMU.GPS(shared)
    # imu_thread = GPS_IMU.IMU(shared)
    # imu_glider_thread = GPS_IMU.IMU_Glider(shared)

    gps_thread.start()
    # imu_thread.start()
    # imu_glider_thread.start()

    return gps_thread#, imu_thread, imu_glider_thread
def main():
    cmd_queue = mp.Queue()           # UI -> main 的指令佇列
    shutdown_evt = mp.Event()        # 唯一的系統級關機事件
    sender_queue = mp.Queue()
    arduino_command_queue = mp.Queue()

    manager = mp.Manager()
    shared = {
        # 🔧 螺旋槳 PID 參數 + 版本/重置旗標
        'prop_kp': manager.Value('d', 10.0),
        'prop_ki': manager.Value('d', 1.0),
        'prop_kd': manager.Value('d', 1.0),
        'prop_pid_ver': manager.Value('i', 0),
        'prop_reset_ver': manager.Value('i', 0),
        
        # —— 新/確認的系統旗標 —— 
        'nav_enabled': manager.Value('i', 0),      # 導航 ON/OFF
        'prop_enabled': manager.Value('i', 0),     # 螺旋槳 ON/OFF
        'rudder_test_enabled': manager.Value('i', 0), # 舵機測試 ON/OFF
        
        # rudder PID參數控制
        'shared_rudder_reset_ver': manager.Value('i', 0),
        'rudder_kp': manager.Value('d', 0.6),
        'rudder_ki' : manager.Value('d', 0.03),
        'rudder_kd' : manager.Value('d', 0.0),
        'rudder_pid_ver' : manager.Value('i', 0), 
        
        'indicator' : manager.dict(),
        'pidTestCmd': manager.Value('d', 0),
        'exceptCourse': manager.Value('d', 0),
        'propellerParameter': manager.Value('d', 0.0),

        'exp_dir' : get_next_experiment_folder(),

        # 感測器
        'windSpeed' : manager.Value('d', 0.0),
        'GPS': manager.dict(),
        'IMU': manager.dict(),
        'IMU_Glider' : manager.dict(),
        'pressure': manager.Value('d', 0.0),
        'dubins_path_history': manager.list(),
        'current_path_history': manager.list(),
        'gps_history_kalman': manager.list(),
        'BatteryVoltage' : manager.Value('d', 0.0),

        'reset_simulation': manager.Value('i', 0),
        'simu_gps': manager.dict({'lat': 0.0, 'lon': 0.0}),
        'rudder_angle': manager.Value('d', 0.0),
        'rudder_update_interval': manager.Value('d', 1.0), 
        'simu_counter': manager.Value('d', 0),
        'dt_real': manager.Value('d', 1.0),
        'success_threshold' : manager.Value('d', 10.0),
        'back_to_goal_dis' : manager.Value('d', 30.0),
        'speed_index' : manager.Value('i', 1),
        'target_speed' : manager.Value('d', 0.5),

        'now_speed': manager.Value('d', 0.0),
        'current_angle': manager.Value('d', 0.0),
        'current_speed': manager.Value('d', 0.0),
        'wg_speed': manager.Value('d', 0),
        'speed_list': manager.list([0 ,0 ,0 ,0 ,0]),
        'Uc': manager.Value('d', 0.0),
        'Vc': manager.Value('d', 0.0),
        'P': manager.list([[1,0],[0,1]]),

        'R': manager.Value('d', 50.0),
        'goal':  manager.list([]), 
        'replan_dis': manager.Value('d', 10.0),

        'dubins_path': manager.list(),
        'current_path': manager.list(),
        'simulated_state': manager.list([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
        'simulated_path' : manager.list(),

        'current_estimate_interval': manager.Value('d', 1.0),
        'kalman_interval' : manager.Value('d', 5.0),
        'path_update_interval': manager.Value('d', 5.0),
        'sensor_poll_interval': manager.Value('d', 0.1),

        'path_version': manager.Value('i', 0),
        'last_path_sent_version': manager.Value('i', 0),

        # ✅ 新增三個「功能旗標」（之後 worker 只讀這三個）
        'nav_enabled': manager.Value('i', 0),
        'prop_enabled': manager.Value('i', 0),
        'rudder_test_enabled': manager.Value('i', 0),
    }
    sensor_thread(shared)

    shared['simu_sender_queue'] = sender_queue
    shared['arduino_command_queue'] = arduino_command_queue


        # 你的 sender（這行用你現有的 SocketSenderThread 即可）
    sender = socket_client.SocketSenderThread(host="localhost", port=5005)
    sender.start()

    # ✅ 啟動指令路由器（只有這裡改旗標/事件）
    threading.Thread(target=command_router, args=(cmd_queue, shutdown_evt, shared, sender),daemon=True).start()
    threading.Thread(target=process_queue_data,args=(shared, sender, cmd_queue), daemon=True).start()
    mp.Process(target=propeller, args=(shared, shutdown_evt), daemon=True).start()

    
    threading.Thread(target=myPosition, args=(shared, sender), daemon=True).start()
    threading.Thread(target=imuLogger, args=(shared,  ), daemon=True).start()
    threading.Thread(target=imuGliderLogger, args=(shared,  ), daemon=True).start()
    threading.Thread(target=gpsLogger, args=(shared,  ), daemon=True).start()
    threading.Thread(target=bridgeSimuQueueToSender, args=(shared, sender), daemon=True).start()

    mp.Process(target=rudderPidTest, args=(shared, shutdown_evt), daemon=True).start()

    nav_process = mp.Process(target=navigationProcess, args=(shared, shutdown_evt),daemon=True)
    nav_process.start()
    # floaterMainArduino = ArduinoReceiver(port="/dev/FloaterArduino", baudrate=9600, shared=shared)
    # floaterMainArduino.start()    
    # gliderMainArduino = ArduinoManager(port="/dev/GliderArduino", baudrate=9600, shared=shared)
    # gliderMainArduino.start()


    while True:
            gps = shared['GPS']
            stats = {
                'wgSpeed':   shared['wg_speed'].value,
                'flowSpeed': shared['current_speed'].value,
                'flowDir':   shared['current_angle'].value,
                'heading':   gps.get('heading', (90 - shared['IMU'].get('angle_yaw', 0.0)) % 360),
                'exceptCourse' : shared['exceptCourse'].value,
                'rudderAngle' : shared['rudder_angle'].value,
                'windSpeed' : shared['windSpeed'].value,
                'BatteryVoltage' : shared['BatteryVoltage'].value,
                'depth' : shared['pressure'].value,
            }
            # print(f"IMU RAW: {(shared['IMU'].get('angle_yaw', 0.0))}")
            # print(f"IMU : {(90 - shared['IMU'].get('angle_yaw', 0.0)) % 360}")
            indicator_dic = dict(shared['indicator'])
            shared['simu_sender_queue'].put(('indicator', indicator_dic))
            # 
            cmd = "rudder@" + str(round(shared['rudder_angle'].value, 2))
            checksum = generateChecksum(cmd)
            rudderCmd = "$" + cmd + "*" + checksum
            shared['arduino_command_queue'].put(rudderCmd)
            print('rudderCmd : ', round(shared['rudder_angle'].value, 2))

            cmd = "propeller@" + str(round(shared['propellerParameter'].value,2))
            checksum = generateChecksum(cmd)
            propellerCmd = "$" + cmd + "*" + checksum
            shared['arduino_command_queue'].put(propellerCmd)
            print('propellerCmd : ', round(shared['propellerParameter'].value,2))
            
            # 
            shared['simu_sender_queue'].put(('updateStats', stats))
            path_version = shared['path_version'].value
            last_path_sent_version = shared['last_path_sent_version'].value
            
            
            if path_version > last_path_sent_version:
                sender.sendQueue.put(("drawSimulate", list(shared['current_path'])))
                shared['last_path_sent_version'].value += 1     

            time.sleep(0.5)


    
        
if __name__ == '__main__':
    main()





