import sys
project_root = r"C:\Users\david ho\Desktop\WG"
if project_root not in sys.path:
    sys.path.insert(0, project_root)
    
import numpy as np
import matplotlib.pyplot as plt
from WG.Model import PID # type: ignore
from WG.Model import waveGenerator # type: ignore
from WG.Model import KalmanEstimateCurrent # type: ignore
from WG.Model import findHeading # type: ignore
from WG.Model import findVirtualTarget # type: ignore
from scipy.integrate import solve_ivp
from WG.Model import wavegliderDynamic # type: ignore

pid_controller = PID.PID()
def wave_glider_simulation():
    # 初始條件
    start = np.array([0, 0, 0])
    goal = np.array([150, 120, 0])
    theta = np.arctan2(goal[1] - start[1], goal[0] - start[0])
    
    glider_state = np.array([
        start[0], start[1], 0, start[2], 0, 0, 0, 0,  
        start[0], start[1], 8, start[2], 0, 0, 0, 0 
    ])
    
    current_speed = 0
    current_angle = 0
    current_angle_d = current_angle
    current_on = 1
    
    timer = 100
    kf_time = 30
    dt = 1
    t_cal = np.linspace(0, dt, 10000)
    R = 30
    
    wg_speed = 1
    
    current_est = []
    total_dubins_paths = [np.empty((0, 2))]
    total_current_paths = [np.empty((0, 2))]
    total_path_index = 0
    change_point = []
    
    with_current = [glider_state]
    without_current = [glider_state]
    
    Uc_est, Vc_est = 0, 0
    
    # 產生波浪力
    waveGenerator.excited_force(U195=10)
    F_wave1 = 10
    # ODE 相關
    P = np.eye(2) * 0.1
    V_sequence = []
    
    #  start, np.append(goal, theta), R, current_speed, wg_speed, current_angle, glider_state[3]
    dubins_path, current_path = findVirtualTarget.findVirtualTarget(
        start, np.append(goal, theta), R, current_speed, wg_speed, current_angle, glider_state[3]
    )

    total_dubins_paths[total_path_index] = np.vstack((total_dubins_paths[total_path_index], dubins_path))
    total_current_paths[total_path_index] = np.vstack((total_current_paths[total_path_index], current_path))
    
    counter = 0
    stat_without_current = glider_state.copy()
    kf_state_c = np.empty((0,16))
    kf_state_no_c = np.empty((0,16))
    
    while True:
        init_state = glider_state
        print(init_state[:2])
        d, heading = findHeading.findHeading(current_path, init_state[:2], dubins_path)
        
        if d > 7:
            change_point.append(init_state[:2])
            current_angle = np.degrees(np.arctan2(Vc_est, Uc_est))
            current_speed = np.sqrt(Uc_est**2 + Vc_est**2)
            theta = np.arctan2(goal[1] - init_state[1], goal[0] - init_state[0])
            
            dubins_path, current_path = findVirtualTarget.findVirtualTarget(
                np.array([init_state[0], init_state[1], init_state[3]]),
                np.array([goal[0], goal[1], theta]),
                R, dt, current_speed, wg_speed, current_angle, init_state[3]
            )
            
            total_dubins_paths.append(dubins_path)
            total_current_paths.append(current_path)
            total_path_index += 1
            continue
        
        rudder_angle = pid_controller.compute(init_state, heading, dt)
        
        # 有流場
        dynamic_with_rudder = lambda t, state: wavegliderDynamic.wavegliderDynamic(
            t, state, rudder_angle, t_cal, F_wave1, current_angle, current_on
        )
        sol = solve_ivp(dynamic_with_rudder, [counter * dt, (counter + 1) * dt], init_state, method='RK45')
        glider_state = sol.y[:, -1]
        
        kf_state_c = np.vstack((kf_state_c, sol.y.T))

        with_current.append(glider_state)
        
        # 無流場
        dynamic_with_rudder = lambda t, state: wavegliderDynamic.wavegliderDynamic(
            t, state, rudder_angle, t_cal, F_wave1, current_angle, 0
        )
        sol = solve_ivp(dynamic_with_rudder, [counter * dt, (counter + 1) * dt], stat_without_current, method='RK45')
        
        if len(kf_state_no_c) == 0:  # 如果是空的
            kf_state_no_c = sol.y.T  # 直接賦值，不用 vstack
        else:
            kf_state_no_c = np.vstack((kf_state_no_c, sol.y.T))
            
        stat_without_current = sol.y[:, -1]
        
        if counter % kf_time == 0:
            Uc_est, Vc_est, P, V = KalmanEstimateCurrent.KalmanEstimateCurrent(kf_state_c, kf_state_no_c, kf_time, Uc_est, Vc_est, P)
            V_sequence.append(V)
            current_est.append([Uc_est, Vc_est])
            kf_state_c = np.empty((0,16))
            kf_state_no_c = np.empty((0,16))
            stat_without_current = glider_state
            wg_speed = np.mean(V_sequence[-5:]) if len(V_sequence) > 5 else np.mean(V_sequence)
        
        counter += dt
        
        if np.linalg.norm(glider_state[:2] - goal[:2]) < 7 or abs(counter - timer) < 0.1:
            print("到達目標")
            print(glider_state)
            break
    
    # 畫圖
    fig, ax = plt.subplots()
    ax.grid(True)
    ax.scatter(*start[:2], label="Start", linewidth=4)
    ax.scatter(*goal[:2], label="Goal", linewidth=4)
    ax.quiver(0, 0, 30 * np.cos(np.radians(current_angle_d)), 30 * np.sin(np.radians(current_angle_d)), color='green', linewidth=3)
    
    for i, path in enumerate(total_current_paths):
        color = 'gray' if i < len(total_dubins_paths) - 1 else 'blue'
        ax.plot(path[:, 0], path[:, 1], linewidth=2, color=color)
    
    for point in change_point:
        ax.scatter(*point, s=50, color='b')
    
    with_current_arr = np.array(with_current)
    ax.plot(with_current_arr[:, 0], with_current_arr[:, 1], linewidth=3, color='r', label="Trajectory")
    
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory with Current")
    ax.set_aspect('equal')
    ax.legend()
    plt.show()
    
wave_glider_simulation()
