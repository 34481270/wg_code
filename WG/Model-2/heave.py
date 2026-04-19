import numpy as np

def heave(t, state, A_heave, B_heave, A_wp, tau_tether, tau_wave=0):
    rho = 1025  # 海水密度 (kg/m³)
    g = 9.81  # 重力加速度 (m/s²)
    m = 60  # 質量
    
    # 提取狀態變數
    z = state[0]  # 位移
    zdot = state[1]  # 速度
    
    # 質量項
    M = m - A_heave
    
    # 阻尼項
    B = B_heave
    
    # 恢復力項 (浮力和重力)
    G = rho * g * A_wp
    
    # 外力
    tau = 30000 * np.sin(0.62 * t) + tau_tether[2]
    
    # 計算加速度
    zddot = (tau + B * zdot - G * z) / M
    
    # 返回狀態微分向量
    return np.array([zdot, zddot])