import sys
project_root = r"C:\Users\david ho\Desktop\WG"
if project_root not in sys.path:
    sys.path.insert(0, project_root)
    
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from WG.Model import PID  # type: ignore
from WG.Model import findHeading  # type: ignore
from WG.Model import waveGenerator  # type: ignore
from WG.Model import findVirtualTarget  # type: ignore
from WG.Model import wavegliderDynamic # type: ignore
from WG.sensor import gps_utils #  type: ignore

pid_controller = PID.PID()
start = np.array([0, 0, 3])
goal = np.array([150, -120])
theta = np.arctan2(goal[1] - start[1], goal[0] - start[0])
glider_state = np.array([
    start[0], start[1], 0, start[2], 0, 0, 0, 0,
    start[0], start[1], 8, start[2], 0, 0, 0, 0
])

rudder_angle = 0
current_angle = 135
current_on = 1
current_speed = 0.2
wg_speed = 1
R = 40

counter = 0
dt = 1
time = 100
t_cal = np.linspace(0, time, 10000)

# [25 ,121]
start_lat = 25
start_lon = 121

with_current = np.empty([0, 16])

waveGenerator.excited_force(U195=10)
dubins_path, current_path = findVirtualTarget.findVirtualTarget(
    start, np.append(goal, theta), R, current_speed, wg_speed, current_angle, glider_state[3])

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
dubins_path_latlon = np.array(dubins_path_latlon)
current_path_latlon = np.array(current_path_latlon)

    # waveglider_func = lambda t, state: wavegliderDynamic.wavegliderDynamic(t, state, 20, t_cal, F_wave1, current_angle, 0)
    # simulated_state2 = solve_ivp(waveglider_func, [simu_counter, simu_counter + dt_real], np.array(simulated_state), method='RK45')

while counter < time:
    d, heading = findHeading.findHeading(current_path, glider_state[:2], dubins_path)
    rudder_angle = pid_controller.compute(glider_state[3], heading, dt)


    waveglider_func = lambda t, state: wavegliderDynamic.wavegliderDynamic(
        t, state, rudder_angle, t_cal, F_wave1, current_angle, current_on)
    sol_with_current = solve_ivp(
        waveglider_func, [counter * dt, (counter + 1) * dt], glider_state, method='RK45')
    with_current = np.vstack((with_current, sol_with_current.y.T))
    glider_state = sol_with_current.y[:, -1]
    counter += dt
    print(counter)

floater1_pos, floater1_vel = with_current[:, :4], with_current[:, 4:8]
glider1_pos, glider1_vel = with_current[:, 8:12], with_current[:, 12:16]


floater_path_xy = floater1_pos[:,:2] + np.array([x1, y1])

floater_path_latlon = [
    list(gps_utils.utm_to_gps(x, y))  # 轉成 [lat, lon]
    for x, y in floater_path_xy
]
floater_path_latlon = np.array(floater_path_latlon)


# 畫 2D 路徑圖
fig2d, ax2d = plt.subplots()
ax2d.grid(True)
# ax2d.scatter(*start[:2], label="Start", linewidth=4)
# ax2d.scatter(*goal[:2], label="Goal", linewidth=4)
ax2d.plot(floater_path_latlon[:, 0], floater_path_latlon[:, 1], linewidth=3, color='g', label="Trajectory")
# ax2d.plot(dubins_path_latlon[:, 0], dubins_path_latlon[:, 1], linewidth=3, color='b', label="Planned Path")
# ax2d.plot(current_path_latlon[:, 0], current_path_latlon[:, 1], linewidth=3, color='r', label="Planned Path")
ax2d.set_xlabel("X (m)")
ax2d.set_ylabel("Y (m)")
ax2d.set_title("Trajectory with Current")
ax2d.set_aspect('equal')
ax2d.legend()
plt.show()