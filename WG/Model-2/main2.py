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

stop_event = mp.Event()
stop_event.set()

rudder_pid_stop_event = mp.Event()
rudder_pid_stop_event.set()

propeller_pid_stop_event = mp.Event()
propeller_pid_stop_event.set()


def bearing(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    theta = math.atan2(x, y)
    print("期望航向：")
    print((math.degrees(theta) + 360) % 360)
    return (math.degrees(theta) + 360) % 360



def rudderPidTest(shared,rudder_pid_stop_event, rudder_pid):
    last_ver = -1
    last_reset_ver = -1

    while True:
        last_rudder_update_time = time.time()
        while not rudder_pid_stop_event.is_set():
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


            now = time.time()
            if now - last_rudder_update_time >= shared['rudder_update_interval'].value:
                cmd = shared['pidTestCmd'].value
                rudder_angle = rudder_pid.compute(shared['GPS'].get('heading', (90 - shared['IMU'].get('angle_yaw', 0.0)) % 360), cmd, shared['dt_real'].value)
                shared['rudder_angle'].value = rudder_angle
                # shared['arduino_command_queue'].put(f"rudder@{rudder_angle}")
                
                dt_real = now - last_rudder_update_time
                shared['dt_real'].value = dt_real
                last_rudder_update_time = now
                shared['simu_sender_queue'].put(("course_angle", shared['GPS'].get('heading', (90 - shared['IMU'].get('angle_yaw', 0.0)) % 360)))
                time.sleep(0.01)

def navigationProcess(shared, stop_event, rudder_pid, propellerPid): 
    last_ver = -1
    last_reset_ver = -1
    while True:
        last_rudder_update_time = time.time()
        flow_blocked = False
        print("2222")

        while not stop_event.is_set():
            print("111111")
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


            shared['indicator']['navigation'] = 1
            goal = list(shared['goal'])
            now = time.time()

            gps = dict(shared['GPS'])
            lat = gps.get('lat', 0.0)
            lon = gps.get('lon', 0.0)
            shared['gps_history_kalman'].append([lat, lon])

            if now - last_rudder_update_time >= shared['rudder_update_interval'].value:
                if shared['reset_simulation'].value:
                    simulated_path = list(shared['simulated_path'])

                    simu_lat = shared['simu_gps']['lat']
                    simu_lon = shared['simu_gps']['lon']
                    x0, y0 = gps_utils.gps_to_utm(simu_lat, simu_lon)

                    simu_gps_path = []
                    for i, xy in enumerate(simulated_path[:]):
                        x_rel, y_rel = xy
                        x = x0 + x_rel
                        y = y0 + y_rel
                        lat, lon = gps_utils.utm_to_gps(x, y)
                        simu_gps_path.append([lat, lon])

                    shared['simu_sender_queue'].put(("drawSimulate", simu_gps_path))

                    gps_history = list(shared['gps_history_kalman'])
                    utm_history = []
                    for lat, lon in gps_history:
                        x, y = gps_utils.gps_to_utm(lat, lon)
                        utm_history.append([x, y])

                    Uc_est, Vc_est, P_new, V = KalmanEstimateCurrent.KalmanEstimateCurrent(
                        np.array(utm_history),
                        np.array(simu_gps_path),
                        shared['kalman_interval'].value,
                        shared['Uc'].value,
                        shared['Vc'].value,
                        np.array(shared['P'])
                    )

                    shared['Uc'].value = Uc_est
                    shared['Vc'].value = Vc_est
                    shared['P'][:] = P_new.tolist()
                    shared['current_speed'].value = np.hypot(shared['Uc'].value, shared['Vc'].value)
                    shared['current_angle'].value = np.degrees(np.arctan2(shared['Vc'].value, shared['Uc'].value))

                    # ??
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
                        print(f"findHeading ???? {e}")
                        last_rudder_update_time = now
                        time.sleep(0.01)
                        continue

                allow_flow_block = (
                    (min_dist is not None) and 
                    (min_dist >= shared['replan_dis'].value)
                )

                current_speed = shared['current_speed'].value
                wg_speed      = shared['wg_speed'].value

                if not flow_blocked:
                    if allow_flow_block and (current_speed > wg_speed):
                        flow_blocked = True
                        shared['simu_sender_queue'].put(("State", 'æµé€éå¤§'))
                else:
                    if current_speed <= wg_speed * 1.05:
                        flow_blocked = False
                        shared['simu_sender_queue'].put(("State", 'æµé€ç·©å'))
                        pathPlanningProcess(shared)
                        shared['simu_sender_queue'].put(("drawTrajectory", list(shared['current_path'])))
                        shared['last_path_sent_version'].value += 1

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
                        print("asdjfkajsdlkfjalskdfs")

                        rudder_angle = rudder_pid.compute(course_angle, desired_heading, shared['dt_real'].value)
                        shared['rudder_angle'].value = rudder_angle
                else:
                    if heading_dubins is not None:
                        shared['exceptCourse'].value = heading_dubins
                        rudder_angle = rudder_pid.compute(course_angle, heading_dubins, shared['dt_real'].value)
                        shared['rudder_angle'].value = rudder_angle

                modelSimulationProcess(shared)

                dt_real = now - last_rudder_update_time
                shared['dt_real'].value = dt_real
                last_rudder_update_time = now

                try:
                    dis2goal = geodesic([gps.get('lat', 0.0), gps.get('lon', 0.0)], [goal[1], goal[0]]).meters
                    if dis2goal < shared['success_threshold'].value:
                        shared['simu_sender_queue'].put(("State", '?????'))
                        propellerPid.setZero()
                        shared['indicator']['navigation'] = 0
                        shared['indicator']['propeller'] = 0
                        stop_event.set()
                except Exception:
                    pass

                if (min_dist is not None) and (min_dist >= shared['replan_dis'].value or list(shared['goal']) != goal):
                    shared['simu_sender_queue'].put(("State", 'èª¤å·®éå¤§éæ°è¦å'))
                    pathPlanningProcess(shared)

            time.sleep(0.01)

        # try:
        #     goal = list(shared['goal'])
        #     gps = dict(shared['GPS'])
        #     dis2goal = geodesic([gps.get('lat', 0.0), gps.get('lon', 0.0)], [goal[1], goal[0]]).meters
        #     if dis2goal > shared['success_threshold'].value:
        #         stop_event.clear()
        # except:
        #     pass

        time.sleep(0.5)





def pathPlanningProcess(shared): 
    gps = shared['GPS']
    while True:
        start_lat = gps.get('lat', None)
        start_lon = gps.get('lon', None)
        if start_lat is not None and start_lon is not None:
            break
        print(" ç­å¾ GPS è³æä¸­")
        time.sleep(0.5)

    goal = list(shared['goal'])  # [lat, lon]
    if len(goal) < 2:
        return


    
    x, y = gps_utils.gps_to_relative_xy(start_lat, start_lon, goal[1], goal[0])
    theta = np.arctan2(y, x)

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
    # è½åçµå° UTM åè½ GPS ç¶ç·¯åº¦
    x0, y0 = gps_utils.gps_to_utm(start_lat, start_lon)
    dubins_path_xy = dubins_path + np.array([x0, y0])
    
    x1, y1 = gps_utils.gps_to_utm(start_lat, start_lon)
    current_path_path_xy = current_path + np.array([x1, y1])

    dubins_path_latlon = [
        list(gps_utils.utm_to_gps(x, y))  # è½æ [lat, lon]
        for x, y in dubins_path_xy
    ]
    
    current_path_latlon = [
        list(gps_utils.utm_to_gps(x, y))  # è½æ [lat, lon]
        for x, y in current_path_path_xy
    ]
    
    shared['dubins_path'][:] = dubins_path_latlon
    shared['current_path'][:] = current_path_latlon
    
    shared['current_path_history'].append(current_path_latlon)
    shared['dubins_path_history'].append(dubins_path_latlon)
    shared['path_version'].value += 1
    log_path_planning(shared)
    shared['simu_sender_queue'].put(("State", "è·¯å¾è¦åå®æ"))                

    

def modelSimulationProcess(shared):
    rudder_angle = shared['rudder_angle'].value # åå¾èµè§
    dt_real = shared['dt_real'].value # åå¾ä¸æ¬¡è¨ç®è§åº¦çæééé
    current_angle = shared['current_angle'].value # åå¾æµå ´è§åº¦
    simu_counter = shared['simu_counter'].value # èå¾ç®åæ¨¡æ¬ç¸½ç§æ¸
    shared['simu_counter'].value += dt_real # å°æ¨¡æ¬ç¸½ç§æ¸å ä¸é€æ¬¡æ¨¡æ¬æé
    
    t_cal = np.linspace(0, dt_real, 1000)
    tau_wave = waveGenerator.excited_force(U195=10)
    
    simulated_state = list(shared['simulated_state'])  # â å¿«ç§å¯æ¬
    
    # å¸¶å¥æ¨¡æ¬
    waveglider_func = lambda t, state: wavegliderDynamic.wavegliderDynamic(t, state, rudder_angle, t_cal, tau_wave, current_angle, 0)
    simulated_state2 = solve_ivp(waveglider_func, [simu_counter, simu_counter + dt_real], np.array(simulated_state), method='RK45')
    
    shared['simulated_state'][:] = list(simulated_state2.y[:,-1]) # å°æ¨¡æ¬çæ€å¾ç€æå­å¥
    shared['simulated_path'].extend(np.stack((simulated_state2.y[0], simulated_state2.y[1]), axis=-1)) # å°æ¨¡æ¬çæ€æç€æå­å¥è·¯å¾
    if shared['simu_counter'].value >= shared['kalman_interval'].value:
        shared['reset_simulation'].value = 1
        
    time.sleep(0.01)


    

def sensor_thread(shared):
    gps_thread = GPS_IMU.GPS(shared)
    # imu_thread = GPS_IMU.IMU(shared)
    # imu_glider_thread = GPS_IMU.IMU_Glider(shared)

    gps_thread.start()
    # imu_thread.start()
    # imu_glider_thread.start()

    return gps_thread#, imu_thread, imu_glider_thread

def process_queue_data(shared, sender, propellerPid, rudder_pid):
    
    def setGoal(data):
        try:
            lat = data['lat']
            lng = data['lng']
            shared['goal'][:] = [lng, lat]
            # éæ°è¦å
            # pathPlanningProcess(shared)

            # é€ç¥ UI æ´æ°è·¯å¾
            # sender.sendQueue.put(("drawTrajectory", list(shared['current_path'])))
            # shared['last_path_sent_version'].value += 1
            print(f"â setGoalï¼æ´æ°ç®æ¨é»çº lat={lat}, lon={lng}")
        except Exception as e:
            print(f"â setGoal é¯èª¤ï¼{e}")


    def experimentStart(data=None):
        global stop_event, rudder_pid_stop_event
        print("adsjfklasjdkfjasldfj")
        rudder_pid_stop_event.set()
        if len(shared['goal']) == 0:
            sender.sendQueue.put(("noGoal", 0))
            return

        if not stop_event.is_set():
            print("â ï¸ å¯¦é©å·²å¨é€²è¡ä¸­")
            return

        # åè·¯å¾è¦åä¸¦é€ç¥ UI
        pathPlanningProcess(shared)

        sender.sendQueue.put(("drawTrajectory", list(shared['current_path'])))
        shared['last_path_sent_version'].value += 1

        # ååå°èªå­é€²ç¨
        shared['sys_armed'].value = 1
        stop_event.clear()

        
        shared['simu_gps']['lat'] = shared['GPS']['lat']
        shared['simu_gps']['lon'] = shared['GPS']['lon']
        sender.sendQueue.put(('State', "å°èªä¸­"))
        print('â å¯¦é©éå§')
        

    
    def experimentFinish(data=None):
        global stop_event, rudder_pid_stop_event
        try:
            propellerPid.setZero()
            shared['indicator']['propeller'] = 0
            shared['sys_armed'].value = 0
            stop_event.set()
            rudder_pid_stop_event.set()
            clear_shared_for_next_experiment(shared)
            shared['propellerParameter'].value = 0
            shared['shared_rudder_reset_ver'].value += 1
            

            shared['indicator']['navigation'] = 0
            sender.sendQueue.put(('State', "å¯¦é©æåçµæ"))
        except Exception as e:
            print(f"? experimentFinish ä¾å¤{e}")
        
    def UI_is_ready(data):
        sender.UI_is_ready()

    def pid_parameter_adjust(data):
        global rudder_pid_stop_event, propeller_pid_stop_event
                
        if data[0] == 'propeller':
            propellerPid.propeller_pid_parameter_adjust(data[1], data[2], data[3])
            rudder_pid_stop_event.set()
            propeller_pid_stop_event.clear()
        elif data[0] == 'rudder':
            shared['rudder_kp'].value = data[1]
            shared['rudder_ki'].value = data[2]
            shared['rudder_kd'].value = data[3]
            shared['rudder_pid_ver'].value += 1

            shared['pidTestCmd'].value = data[6]
            propeller_pid_stop_event.clear()
            rudder_pid_stop_event.clear()
        
        elif data[0] == 'general':
            shared['target_speed'].value = data[4]
            shared['rudder_angle'].value = data[5]
            # shared['arduino_command_queue'].put(f"rudder@{data[5]}")
            rudder_pid_stop_event.set()
            propeller_pid_stop_event.clear()

    # def pidTestStart(data):
    #     global rudder_pid_stop_event
    #     rudder_pid_stop_event.clear()


    def pidTestFinish(data):
        global rudder_pid_stop_event, propeller_pid_stop_event
        rudder_pid_stop_event.set()
        propeller_pid_stop_event.set()
        shared['indicator']['propeller'] = 0
        shared['propellerParameter'].value = 0
        shared['shared_rudder_reset_ver'].value += 1
        propellerPid.setZero()
        


    
    handler_map = {
        'setGoal' : setGoal,
        'experimentStart' : experimentStart,
        'experimentFinish' : experimentFinish,
        'UI_is_ready' : UI_is_ready,
        'pid' : pid_parameter_adjust,
        # 'pidTestStart' : pidTestStart,
        'pidTestFinish': pidTestFinish,
    }

    while True:
        try:
            item = sender.recvQueue.get(timeout=1)

            if isinstance(item, tuple) and len(item) == 2:
                cmd, data = item
                handler = handler_map.get(cmd)
                if handler:
                    handler(data)
                else:
                    print(f"â ï¸ æªç¥æä»¤ï¼{cmd}")
        except Empty:
            continue
        
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
            # ??????? header
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


def log_path_planning(shared):
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    goal = list(shared['goal'])
    current_path = list(shared['current_path'])
    dubins_path = list(shared['dubins_path'])

    exp_dir = shared['exp_dir']  

    # å²å­ current_path
    current_file = os.path.join(exp_dir, f"current_path_{timestamp}.xtxt")
    with open(current_file, "w", encoding="utf-8") as f:
        f.write(f"# Timestamp: {timestamp}\n")
        f.write(f"# Goal: {goal}\n")
        f.write("lat,lon\n")
        for lat, lon in current_path:
            f.write(f"{lat:.8f},{lon:.8f}\n")

    # å²å­ dubins_path
    dubins_file = os.path.join(exp_dir, f"dubins_path_{timestamp}.txt")
    with open(dubins_file, "w", encoding="utf-8") as f:
        f.write(f"# Timestamp: {timestamp}\n")
        f.write(f"# Goal: {goal}\n")
        f.write("lat,lon\n")
        for lat, lon in dubins_path:
            f.write(f"{lat:.8f},{lon:.8f}\n")



def get_next_experiment_folder(log_root="path_logs"):
    """
    å¨ path_logs ä¸å»ºç«ä¸€å€æ°ç experiment_N è³æå¤¾ã€
    å¦æå·²å­å¨ experiment_1, experiment_2ï¼å°±æå»ºç« experiment_3ã€
    """
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

def propeller(shared, propeller_pid, propeller_pid_stop_event, rudder_pid_stop_event, stop_event, sender):
    while True:
        propeller_on = False  # ç®åèºææ§³ç€æï¼åå§æ¯ééï¼
        while not stop_event.is_set() or not propeller_pid_stop_event.is_set():
            print("2")
            gps = shared['GPS']
            GPS_speed = gps.get('speed', 0)
            history_speed = shared['speed_list']
            shared['speed_index'].value = (shared['speed_index'].value + 1) % 5
            idx = shared['speed_index'].value
            history_speed[idx] = GPS_speed
            
            now_speed = sum(history_speed) / len(history_speed)
            
            if not propeller_pid_stop_event.is_set() and rudder_pid_stop_event.is_set():
                shared['simu_sender_queue'].put(("course_angle", now_speed)) # é€éä¸€æ¨£é€"course_angle"æ¯å çºé€éå»çå°±æ¯æçé¿æ æ¶çå€årudder or propeller


            shared['wg_speed'].value = GPS_speed
            print('wg_speed ', shared['wg_speed'].value)
            target_speed = shared['target_speed'].value

            # â æ§å¶éè¼¯
            if now_speed < target_speed - 0.3 and not propeller_on:
                sender.sendQueue.put(('State', "èºæ§³åå"))
                
                print("ð€ èºææ§³ååï¼")
                propeller_on = True
                
            elif now_speed > target_speed + 0.2 and propeller_on:
                sender.sendQueue.put(('State', "èºæ§³éé"))
                print("ð èºææ§³ééï¼")
                # propellerPid.setZero()
                propeller_on = False

            # â PID æ§å¶é¦¬éé€åº¦ï¼åªæå¨èºææ§³ ON ææåï¼
            if propeller_on:
                # ç®æ¨é€åº¦å¯ä»¥è¨­å®çº 0.7 m/s (ä»æ¼0.5å1.0ä¹éï¼ç©©å®å°±å¥½)
                shared['indicator']['propeller'] = 1
                error = target_speed - now_speed
                print('nowSpeed ', now_speed)
                print('target_speed ', target_speed)
                print('error ', target_speed - now_speed)
                motor_cmd = propeller_pid.update(error)
                shared['propellerParameter'].value = motor_cmd

                # shared['arduino_command_queue'].put(f"propeller@{(motor_cmd)}")
            else:
                # å¦æèºææ§³ééï¼å°±å³0çµ¦Arduino
                shared['indicator']['propeller'] = 0
                shared['propellerParameter'].value = 0
            
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

    def propeller_pid_parameter_adjust(self, Kp, Ki ,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        print(self.Kp)
        print(self.Ki)
        print(self.Kd)
        print('propeller PIDåæ¸è¨­å®å®æ')

    def update(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        # é²æ­¢é¤ä»¥0
        if dt <= 0.0:
            dt = 1e-16

        # PIDä¸é è¨ç®
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
    shared['IMU'].get('angle_yaw', 0.0)
    shared['rudder_angle'].value = 0.0
    shared['simulated_state'][:] = [0.0, 0.0, 0.0, shared['IMU'].get('angle_yaw', 0.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, shared['IMU_Glider'].get('angle_yaw', 0.0), 0.0, 0.0, 0.0, 0.0]
    shared['simulated_path'][:] = []
    shared['simu_counter'].value = 0
    shared['speed_list'][:] = [0, 0, 0, 0, 0]
    shared['speed_index'].value = 1
    shared['wg_speed'].value = 0
    shared['now_speed'].value = 0.0
    shared['dubins_path'][:] = []
    shared['current_path'][:] = []
    shared['path_version'].value = 0
    shared['last_path_sent_version'].value = 0
    shared['current_speed'].value = 0
    shared['current_angle'].value = 0
    shared['goal'][:] = []

def generateChecksum(cmd):
    checksum = 0
    for ch in cmd:
        checksum ^= ord(ch)
    return f"{checksum:02X}"


def main():
    sender_queue = mp.Queue()
    arduino_command_queue = mp.Queue()

    manager = mp.Manager()
    shared = {
        'shared_rudder_reset_ver': manager.Value('i', 0),


        'rudder_kp': manager.Value('d', 0.6),
        'rudder_ki' : manager.Value('d', 0.03),
        'rudder_kd' : manager.Value('d', 0.0),
        'rudder_pid_ver' : manager.Value('i', 0), 
        
        'indicator' : manager.dict(),
        'pidTestCmd': manager.Value('d', 0),
        'exceptCourse': manager.Value('d', 0),
        'propellerParameter': manager.Value('d',0.0),

        'exp_dir' : get_next_experiment_folder(),
        
        'windSpeed' : manager.Value('d', 0.0),
        'GPS': manager.dict(),
        'IMU': manager.dict(),
        'IMU_Glider' : manager.dict(),
        'pressure': manager.Value('d', 0.0),
        'dubins_path_history': manager.list(), # ç¡æµå ´è·¯å¾è¦åæ­·å²è³æ
        'current_path_history': manager.list(), # ææµå ´è·¯å¾è¦åæ­·å²è³æ
        'gps_history_kalman': manager.list(), # å²å­çå¯¦GPS
        'BatteryVoltage' : manager.Value('d', 0.0),
        
        # æ§å¶åæ¸
        'reset_simulation': manager.Value('i', 0), # æ±ºå®æ¯å¦éç½®æ¨¡æ¬ç€æ
        'simu_gps': manager.dict({'lat': 0.0, 'lon': 0.0}), # æ¨¡æ¬éå§æçç¬¬ä¸€å€ä½ç½®
        'rudder_angle': manager.Value('d', 0.0), # å²å­è¨ç®åºçèµæ¿è§åº¦ è®å¶ä»processä½¿ç¨
        'rudder_update_interval': manager.Value('d', 1.0), # å¹¾ç§è¨ç®ä¸€æ¬¡èµæ¿è§åº¦
        'simu_counter': manager.Value('d', 0), # è¨ç®æ¨¡æ¬å¹¾ç§äº
        'dt_real': manager.Value('d', 1.0), # æ¯æ¬¡æ¨¡æ¬çå€é
        'success_threshold' : manager.Value('d', 10.0), # èçµé»çè·é¢
        'speed_index' : manager.Value('i', 1),
        'target_speed' : manager.Value('d', 0.5), # unit m/s
        
                
        # ç€æä¼°æ¸¬
        'now_speed': manager.Value('d', 0.0),
        'current_angle': manager.Value('d', 0.0),
        'current_speed': manager.Value('d', 0.0),
        'wg_speed': manager.Value('d', 0),
        'speed_list': manager.list([0 ,0 ,0 ,0 ,0]),
        'Uc': manager.Value('d', 0.0),
        'Vc': manager.Value('d', 0.0),
        'P': manager.list([[1,0],[0,1]]),
        
        # æ¨¡æ¬åæ¸
        'R': manager.Value('d', 50.0),
        'goal':  manager.list([]), 
        'replan_dis': manager.Value('d', 10.0),
        
        # è¨éèå±äº«
        'dubins_path': manager.list(),
        'current_path': manager.list(),
        'simulated_state': manager.list([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
        'simulated_path' : manager.list(),

        # æéééè¨­å®
        'current_estimate_interval': manager.Value('d', 1.0),
        'kalman_interval' : manager.Value('d', 30.0),
        'path_update_interval': manager.Value('d', 5.0),
        'sensor_poll_interval': manager.Value('d', 0.1),
        
        # è·¯å¾çæ¬æ§å¶
        'path_version': manager.Value('i', 0),              # è¡¨ç¤ºç®åçè·¯å¾æ¯ç¬¬å¹¾ç
        'last_path_sent_version': manager.Value('i', 0),   # è¡¨ç¤ºä¸ä¸€æ¬¡å³é€çè·¯å¾æ¯ç¬¬å¹¾ç
        
    }
    shared['simu_sender_queue'] = sender_queue
    shared['arduino_command_queue'] = arduino_command_queue

    exp_dir = get_next_experiment_folder()
    shared['exp_dir'] = exp_dir
    
    # åå
    propellerPid = propeller_pid(Kp=10, Ki=1, Kd=1)
    rudder_pid = PID.PID()
    sender = socket_client.SocketSenderThread(host="localhost", port=5005) # 192.168.1.200
    sender.start()
    threading.Thread(target=process_queue_data, args=(shared, sender,propellerPid, rudder_pid), daemon=True).start()
    threading.Thread(target=myPosition, args=(shared, sender), daemon=True).start()
    threading.Thread(target=imuLogger, args=(shared,  ), daemon=True).start()
    threading.Thread(target=imuGliderLogger, args=(shared,  ), daemon=True).start()
    threading.Thread(target=gpsLogger, args=(shared,  ), daemon=True).start()
    threading.Thread(target=bridgeSimuQueueToSender, args=(shared, sender), daemon=True).start()
    propeller_thread = threading.Thread(target=propeller, args=(shared, propellerPid, propeller_pid_stop_event, rudder_pid_stop_event, stop_event, sender), daemon=True)
    propeller_thread.start()
    nav_process = mp.Process(target=navigationProcess, args=(shared, stop_event, rudder_pid, propellerPid))
    nav_process.start()
    mp.Process(target=rudderPidTest, args=(shared,rudder_pid_stop_event, rudder_pid)).start()
    floaterMainArduino = ArduinoReceiver(port="/dev/FloaterArduino", baudrate=9600, shared=shared)
    floaterMainArduino.start()    
    gliderMainArduino = ArduinoManager(port="/dev/GliderArduino", baudrate=9600, shared=shared)
    gliderMainArduino.start()

    sensor_thread(shared) #,imu_thread, imu_glider_thread

    
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
        cmd = "rudder@" + str(shared['rudder_angle'].value)
        checksum = generateChecksum(cmd)
        rudderCmd = "$" + cmd + "*" + checksum
        shared['arduino_command_queue'].put(rudderCmd)

        cmd = "propeller@" + str(shared['propellerParameter'].value)
        checksum = generateChecksum(cmd)
        propellerCmd = "$" + cmd + "*" + checksum
        shared['arduino_command_queue'].put(propellerCmd)
        # 
        shared['simu_sender_queue'].put(('updateStats', stats))
        path_version = shared['path_version'].value
        last_path_sent_version = shared['last_path_sent_version'].value
        
        if path_version > last_path_sent_version:
            sender.sendQueue.put(("drawTrajectory", list(shared['current_path'])))
            shared['last_path_sent_version'].value += 1     

        time.sleep(0.5)


    
        
if __name__ == '__main__':
    main()





