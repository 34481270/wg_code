import numpy as np

def floater(t, state, tau_tether):
    # 參數定義
    # print(tau_tether)
    F_m = 60  # 質量
    F_X_du = 0  # 外加質量u係數
    F_Y_dv = 0  # 外加質量v係數
    F_Y_dr = 0  # 外加質量r係數
    F_N_dr = 104.65  # 外加質量N方向係數

    F_x_g = 0.2  # Floater 上座標原點到質心的x方向距離
    F_y_g = 0  # Floater 上座標原點到質心的y方向距離
    F_I_z = 0.9  # Floater z軸轉動慣量

    # 狀態變數
    eta = state[:3]  # 位置和姿態向量 [x, y, psi]
    nu = state[3:]   # 速度向量 [u, v, r]

    F_u, F_v, F_r = nu
    
    # 剛體質量矩陣
    M_rb = np.array([
        [F_m, 0, 0],
        [0, F_m, F_m * F_x_g],
        [0, F_m * F_x_g, F_I_z]
    ])
    
    # 附加質量矩陣
    M_a = np.array([
        [F_X_du, 0, 0],
        [0, F_Y_dv, F_Y_dr],
        [0, F_Y_dr, F_N_dr]
    ])
    
    # 剛體科氏力矩陣
    C_rb = np.array([
        [0, 0, 0],
        [0, 0, F_m * F_u],
        [0, 0, F_m * F_x_g * F_u]
    ])
    
    # 附加質量科氏力矩陣
    C_a = np.array([
        [0, 0, 0],
        [0, 0, F_Y_dv * F_u],
        [0, 0, F_Y_dr * F_u]
    ])
    
    # 阻尼矩陣 (線性+非線性)
    D1 = np.array([[-23.5, 0, 0],
                   [0, -23.5, 0],
                   [0, 0, -23.5]])
    
    D2 = np.array([[-34.3, 0, 0],
                   [0, -238, 0],
                   [0, 236, -110]])
    
    # 總質量矩陣
    M = M_rb + M_a
    
    # 計算 dnu
    tau = np.array([tau_tether[0], tau_tether[1], 0])
    dnu = np.linalg.solve(M, tau - np.dot(C_rb, nu) - np.dot(C_a, nu) + np.dot(D1, nu) + np.dot(D2, np.sqrt(np.linalg.norm(nu)) * nu))
    
    # 位置與姿態變化矩陣
    psi = eta[2]
    J = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])
    deta = np.dot(J, nu)
    
    # 返回狀態變化
    return np.concatenate((deta, dnu))
