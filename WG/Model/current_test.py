import sys
project_root = r"C:\Users\david ho\Desktop\WG"
if project_root not in sys.path:
    sys.path.insert(0, project_root)
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from floater import floater
from glider import glider
from heave import heave
from tether import tether
from WG.Model import wavegliderDynamic


def wave_generator(t_cal):
    """模擬波浪力"""
    F_wave1 = np.sin(0.1 * t_cal)
    F_wave2 = np.sin(0.2 * t_cal)
    return F_wave1, F_wave2

# 初始條件 (SI制 MKS)
x0 = np.array([0, 0, 0, np.deg2rad(0), 0, 0, 0, 0,  # 浮體位置與速度
               0, 0, 8, np.deg2rad(0), 0, 0, 0, 0])  # 潛體位置與速度

rudder_angle = 15
current_angle = 30
current_on = 0

time = 50
tspan = [0, time]
t_cal = np.linspace(0, time, 10000)

# 產生波浪力
F_wave1, F_wave2 = wave_generator(t_cal)

# ODE 無流
waveglider_func = lambda t, state: wavegliderDynamic(t, state, rudder_angle, t_cal, F_wave1, current_angle, 0)
sol_without_current = solve_ivp(waveglider_func, tspan, x0, method='RK45', t_eval=np.linspace(0, time, 1000))
without_current = sol_without_current.y.T

# ODE 有流
waveglider_func = lambda t, state: wavegliderDynamic(t, state, rudder_angle, t_cal, F_wave1, current_angle, current_on)
sol_with_current = solve_ivp(waveglider_func, tspan, x0, method='RK45', t_eval=np.linspace(0, time, 1000))
with_current = sol_with_current.y.T

# 提取狀態
floater1_pos, floater1_vel = with_current[:, :4], with_current[:, 4:8]
glider1_pos, glider1_vel = with_current[:, 8:12], with_current[:, 12:16]
floater2_pos, floater2_vel = without_current[:, :4], without_current[:, 4:8]
glider2_pos, glider2_vel = without_current[:, 8:12], without_current[:, 12:16]

# 3D 繪圖
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(floater1_pos[:, 0], floater1_pos[:, 1], floater1_pos[:, 2], label='Floater')
ax.plot(glider1_pos[:, 0], glider1_pos[:, 1], glider1_pos[:, 2], label='Glider')
# 設置標籤
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Wave Glider Motion')
ax.legend()
ax.grid(True)
x_limits = ax.get_xlim()
y_limits = ax.get_ylim()
z_limits = ax.get_zlim()

# 反轉 Z 軸
ax.set_zlim(ax.get_zlim()[::-1])

# 固定 X, Y, Z 軸比例
ax.set_box_aspect([1, 1, 1])
plt.show()

# # 繪製每一瞬間的連線
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_title('Floater & Glider Connection')
# for i in range(0, len(floater1_pos), 50):
#     ax.plot([floater1_pos[i, 0], glider1_pos[i, 0]],
#             [floater1_pos[i, 1], glider1_pos[i, 1]],
#             [floater1_pos[i, 2], glider1_pos[i, 2]], '-o')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.grid(True)
# plt.show()
