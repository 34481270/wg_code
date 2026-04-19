import numpy as np

def KalmanEstimateCurrent(with_current, without_current, time_step, Uc, Vc, P):

    # 計算 GPS 速度 (實際測量值)
    U_GPS = (with_current[-1, 0] - with_current[0, 0]) / time_step
    V_GPS = (with_current[-1, 1] - with_current[0, 1]) / time_step
    V = np.sqrt(U_GPS**2 + V_GPS**2)
    # print('V_GPS = ', V )
    
    # 計算模型預測的速度 (無流場條件下)
    U_model = (without_current[-1, 0] - without_current[0, 0]) / time_step
    V_model = (without_current[-1, 1] - without_current[0, 1]) / time_step
    V = np.sqrt(U_model**2 + V_model**2)
    # print('V_model = ', V )
    

    # 狀態變數初始化
    X = np.array([[Uc], [Vc]])
    # 狀態轉移矩陣
    F = np.eye(2)
    
    # 觀測矩陣
    H = np.eye(2)
    
    # 過程噪聲協方差矩陣
    Q = np.eye(2) * 0.005
    
    # 觀測噪聲協方差矩陣
    R = np.eye(2) * 0.01
    
    # 觀測值 (流速誤差)
    Z = np.array([[U_GPS - U_model], [V_GPS - V_model]])
    
    # ========== 預測步驟 ==========
    X_pred = np.dot(F, X)
    P_pred = np.dot(F, np.dot(P, F.T)) + Q
    
    # ========== 更新步驟 ==========
    K = np.dot(P_pred, np.dot(H.T, np.linalg.inv(np.dot(H, np.dot(P_pred, H.T)) + R)))
    X = X_pred + np.dot(K, (Z - np.dot(H, X_pred)))
    P = np.dot(np.eye(2) - np.dot(K, H), P_pred)
    
    # 更新估計的海流速度
    Uc_est, Vc_est = X.flatten()  # 確保 X 是 (2,)

    return Uc_est, Vc_est, P, V
