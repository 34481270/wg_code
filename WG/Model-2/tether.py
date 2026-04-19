import numpy as np
import time

def wrap_to_pi(angle):
    """將角度歸一化到 [-pi, pi] 範圍內"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def tether(pos1, pos2):
    # 繫繩參數
    k = 1.7e6  # 繫繩剛度 (kg/m)
    g0 = 41  # 潛水滑翔機的濕重 (kg)
    l0 = 10  # 繫繩初始長度 (m)
    
    # 計算位置差
    
    # print('pos1', pos1)
    # print('pos2', pos2)
    delta_pos = np.array(pos2) - np.array(pos1)
    
    
    # 繫繩伸長量
    delta_l = np.sqrt(np.sum(delta_pos[:3] ** 2)) - (g0 / k) - l0
    
    # 計算張力
    T = max(0, k * delta_l)
    
    # 計算繫繩與 x 軸的夾角
    alp = np.arctan2(pos2[1] - pos1[1], pos2[0] - pos1[0])
    
    alp1 = wrap_to_pi(alp - pos1[3])  # pos1[3] 是浮體的偏航角
    alp2 = wrap_to_pi(alp - pos2[3])  # pos2[3] 是滑翔機的偏航角
    
    beta = np.arcsin(np.sqrt(((pos2[1] - pos1[1]) ** 2 + (pos2[0] - pos1[0]) ** 2) / np.sum(delta_pos[:3] ** 2)))
    
    
    # 計算繫繩對浮體與滑翔機的力矩
    tau_floater = np.array([
        T * np.sin(beta) * np.cos(alp1),
        T * np.sin(beta) * np.sin(alp1),
        T * np.cos(beta),
        0
    ])
    
    tau_glider = np.array([
        -T * np.sin(beta) * np.cos(alp2),
        -T * np.sin(beta) * np.sin(alp2),
        -T * np.cos(beta),
        0
    ])
    
    return tau_floater, tau_glider