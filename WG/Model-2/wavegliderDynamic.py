import numpy as np
from WG.Model import floater  # type: ignore
from WG.Model import glider  # type: ignore
from WG.Model import heave  # type: ignore
from WG.Model import tether  # type: ignore
from scipy.interpolate import interp1d
import time

def wavegliderDynamic(t, state, rudder_angle, t_cal, F_wave1, current_angle, current_on):
    """
    Wave Glider 動態模型
    """
    # 插值計算當前波浪力
    # wave_interp = interp1d(t_cal, F_wave1, fill_value="extrapolate")
    # h1 = wave_interp(t)
    h1 = 10
    # print(t)
    
    # 提取狀態變數
    
    floater_pos = state[:4]   # 浮體位置和姿態 [x_F, y_F, z_F, psi_F]
    floater_vel = state[4:8]  # 浮體速度 [u_F, v_F, w_F, r_F]
    glider_pos = state[8:12]  # 潛體位置和姿態 [x_G, y_G, z_G, psi_G]
    glider_vel = state[12:16] # 潛體速度 [u_G, v_G, w_G, r_G]
    
    # 計算繫繩張力
    tau_floater, tau_glider = tether.tether(floater_pos, glider_pos)
    
    # 計算浮體動態
    floater_state = np.array([floater_pos[0], floater_pos[1], floater_pos[3], 
                              floater_vel[0], floater_vel[1], floater_vel[3]])
    d_floater = floater.floater(t, floater_state, tau_floater)
    
    # 計算浮體垂直運動 (Heave)
    floater_heave = heave.heave(t, np.array([floater_pos[2], floater_vel[2]]), 6.58, 0.4, 4, tau_floater, h1)
    
    # 計算潛體動態
    glider_state = np.concatenate((glider_pos, glider_vel))
    d_glider = glider.glider(t, glider_state, tau_glider, rudder_angle, current_angle, current_on)
    
    # 組合浮體與潛體的狀態變化
    d_floater_combined = np.array([
        d_floater[0], d_floater[1], floater_heave[0], d_floater[2], 
        d_floater[3], d_floater[4], floater_heave[1], d_floater[5]
    ])
    
    dxdt = np.concatenate((d_floater_combined, d_glider))
    
    return dxdt