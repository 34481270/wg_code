import numpy as np

def glider(t, state, tau_tether, r_angle, current_angle, current_on):
    G_m = 41  # 質量
    G_X_du = 0  # 外加質量u係數
    G_Y_dv = 0  # 外加質量v係數
    G_Y_dr = 10  # 外加質量r係數
    G_N_dr = 12  # 外加質量N方向係數

    G_x_g = 0  # 質心距離
    G_I_z = 1.9  # 轉動慣量
    
    rho = 1025  # 水密度
    
    # 狀態變數
    eta = state[:4]  # 位置和姿態向量 [x, y, z, psi]
    nu = state[4:9]   # 速度向量 [u, v, w, r]
    
    # 剛體質量矩陣
    M_rb = np.array([
        [G_m, 0, 0, 0],
        [0, G_m, 0, G_m * G_x_g],
        [0, 0, G_m, 0],
        [0, G_m * G_x_g, 0, G_I_z]
    ])
    
    # 附加質量矩陣
    M_a = np.array([
        [G_X_du, 0, 0, 0],
        [0, G_Y_dv, 0, 0],
        [0, 0, G_Y_dr, 0],
        [0, 0, 0, G_N_dr]
    ])
    
    # 剛體科氏力矩陣
    C_rb = np.array([
        [0, 0, 0, 0],
        [0, 0, 0, G_m * nu[0]],
        [0, 0, 0, 0],
        [0, 0, 0, G_m * G_x_g * nu[0]]
    ])
    
    # 附加質量科氏力矩陣
    C_a = np.array([
        [0, 0, 0, 0],
        [0, 0, 0, -G_Y_dv * nu[0]],
        [0, 0, 0, 0],
        [0, 0, 0, -G_Y_dr * nu[0]]
    ])
    
    # 阻尼矩陣
    D_k = np.array([
        [-85.2, 0, 0, 0],
        [0, -370, 0, 0],
        [0, 0, -2550, 0],
        [0, 9.8, 0, -108]
    ])
    
    g = np.array([0, 0, -38.5, 0])
    
    # 旋轉矩陣
    psi = eta[3]
    J = np.array([
        [np.cos(psi), -np.sin(psi), 0, 0],
        [np.sin(psi), np.cos(psi), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # 合成質量矩陣
    M = M_rb + M_a
    
    # 水翼與舵產生的影響
    lambda_ratio = 2
    x_sweep = 7
    rudder_angle = np.deg2rad(18) # angle of attack 翼板的攻角
    
    rudder_angle2 = np.deg2rad(r_angle) # 舵的角度
    
    C_dc = 0.6
    C_d0 = 0.008
    A = 0.113
    # A_rudder = 0.1
    Cl_hydro = ((1.8 * np.pi * lambda_ratio) / (np.cos(np.radians(x_sweep)) * 
                np.sqrt(lambda_ratio**2 / np.cos(np.radians(x_sweep))**4 + 4) + 1.8)) * rudder_angle + (C_dc/lambda_ratio)*rudder_angle**2
    Cd_hydro = C_d0 + Cl_hydro**2 / (0.9 * np.pi * lambda_ratio)
    
    Cl_rudder = ((1.8 * np.pi * lambda_ratio) / (np.cos(np.radians(x_sweep)) * 
            np.sqrt(lambda_ratio**2 / np.cos(np.radians(x_sweep))**4 + 4) + 1.8)) * rudder_angle2 + (C_dc/lambda_ratio)*rudder_angle2**2
    Cd_rudder = C_d0 + Cl_rudder**2 / (0.9 * np.pi * lambda_ratio)
    
    V_hydro = np.sqrt(nu[0]**2 + nu[2]**2)
    V_rudder = np.sqrt(nu[0]**2 + nu[1]**2)
    
    Fl_hydro = 0.5 * rho * Cl_hydro * A * V_hydro**2
    Fd_hydro = 0.5 * rho * Cd_hydro * A * V_hydro**2
    
    Fl_rudder = 0.5 * rho * Cl_rudder * A * V_rudder**2
    Fd_rudder = 0.5 * rho * Cd_rudder * A * V_rudder**2
    
    af = np.arctan2(abs(nu[2]), abs(nu[0]))
    tau_hydrofoil = np.array([
        (Fl_hydro * np.sin(af) - Fd_hydro * np.cos(af)),
        0,
        (np.sign(nu[2]) * Fl_hydro * np.cos(af) + np.sign(nu[2]) * Fd_hydro * np.sin(af)),
        0
    ])
    
    tau_rudder = np.array([0, 0, 0, -Fl_rudder * 0.3])
    
    # tau_current = np.array([7 * np.cos(np.radians(current_angle)), 7 * np.sin(np.radians(current_angle)), 0, 0]) if current_on else np.zeros(4)
    # np.dot(J_r, tau_current)
    J_r = np.array([
        [np.cos(psi), np.sin(psi), 0, 0],
        [-np.sin(psi), np.cos(psi), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # 計算加速度
    dnu = np.linalg.solve(M, tau_tether + tau_rudder + 12*tau_hydrofoil - np.dot(C_rb, nu) - np.dot(C_a, nu) + np.dot(D_k, np.linalg.norm(nu) * nu) - g)
    
    # 位置微分
    deta = np.dot(J, nu)
    
    # 組合微分狀態向量
    return np.concatenate((deta, dnu))
