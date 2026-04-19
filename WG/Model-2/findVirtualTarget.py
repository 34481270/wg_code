import numpy as np
from WG.Model import dubins  # type: ignore

def findVirtualTarget(start, goal, R, current_speed, wg_speed, current_angle, distance_resolution=0.1):
    
    """
    start: [x0, y0, yaw_rad]   # yaw 是「弧度」!!!
    goal : [xg, yg, theta_rad] # theta 也是「弧度」
    R: 轉彎半徑 (m)
    current_speed: 流速 (m/s)
    wg_speed: 載具地面前進速度(期望) (m/s)
    current_angle: 流向(度), 0=東, 90=北
    distance_resolution: 取樣解析度 (m)
    """
    print('start ',start)
    print('goal ',goal)
    print('current_speed ',current_speed)
    print('wg_speed ',wg_speed)
    print('current_angle ',current_angle)
    
    
    
    # --- 1) 速度與方向向量（注意 start[2] 是弧度，不要再 radians 一次） ---
    yaw = start[2]  # rad
    vh = np.array([np.cos(yaw), np.sin(yaw)])  # 船頭方向單位向量
    v_ground  = max(wg_speed, 0.05) * vh

    v_current = current_speed * np.array([
        np.cos(np.radians(current_angle)),
        np.sin(np.radians(current_angle))
    ])

    # 有效推進速度（等效在地面座標的可用速度）
    v_thr = v_ground - v_current
    wg_eff = max(np.linalg.norm(v_thr), 0.05)  # 下限避免 0

    # --- 2) t 上界用幾何估（兩倍安全裕度），別用硬編碼 36000 ---
    dist = np.linalg.norm(goal[:2] - start[:2])
    t_upper = max(30.0, 2.0 * dist / wg_eff)  # 秒
    t_lower = 0.0
    t_mid   = 0.5 * t_upper

    # --- 3) 二分法：找「抵達時間」與「虛擬目標時間漂移」一致的解 ---
    #    虛擬目標 = 目標位置 - (流速向量 * t_mid)
    converged = False
    dubins_result = None
    for _ in range(60):  # 多給一點迭代上限
        virtual_target = goal[:2] - v_current * t_mid
        try:
            dubins_result = dubins.dubins(start, np.append(virtual_target, goal[2]), R)
        except Exception:
            dubins_result = None

        if dubins_result is None:
            # 無法產生 dubins（幾何上不可達等），縮小 t 範圍再試
            t_upper = max(t_mid, 1.0)
            t_mid   = 0.5 * (t_lower + t_upper)
            continue

        total_length = dubins_result[6]  # 第 7 個元素是總長度
        t_wg = total_length / wg_eff

        if abs(t_wg - t_mid) < 5.0:  # 5 秒內視為收斂
            converged = True
            break

        if t_mid > t_wg:
            # 我們假設太久了 → 縮小
            t_upper = t_mid
        else:
            # 我們假設太短了 → 拉長
            t_lower = t_mid

        t_mid = 0.5 * (t_upper + t_lower)

    # 若沒收斂，做個理性的 fallback
    if (not converged) and (dubins_result is not None):
        total_length = dubins_result[6]
        t_mid = np.clip(total_length / wg_eff, t_lower, t_upper)

    # --- 4) 用最終 t_mid 重新算一次 dubins（確保一致性） ---
    virtual_target = goal[:2] - v_current * t_mid
    dubins_result = dubins.dubins(start, np.append(virtual_target, goal[2]), R)

    # --- 5) 將 dubins 結果按距離解析度離散 ---
    def _build_path_from_result(res):
        if len(res) == 10:
            # 帶第三段弧的情況
            center1_arc, M1a, M2a, center2_arc, center1, center2, length, path_type, center3, center3_arc = res
            arc1_len = abs(R * (center1_arc[0] - center1_arc[-1]))
            arc2_len = abs(R * (center2_arc[0] - center2_arc[-1]))
            arc3_len = abs(R * (center3_arc[0] - center3_arc[-1]))

            arc1_theta = np.linspace(center1_arc[0], center1_arc[-1], max(2, int(arc1_len / distance_resolution)))
            arc2_theta = np.linspace(center2_arc[0], center2_arc[-1], max(2, int(arc2_len / distance_resolution)))
            arc3_theta = np.linspace(center3_arc[0], center3_arc[-1], max(2, int(arc3_len / distance_resolution)))

            x1 = center1[0] + R * np.cos(arc1_theta)
            y1 = center1[1] + R * np.sin(arc1_theta)
            x3 = center3[0] + R * np.cos(arc3_theta)
            y3 = center3[1] + R * np.sin(arc3_theta)
            x2 = center2[0] + R * np.cos(arc2_theta)
            y2 = center2[1] + R * np.sin(arc2_theta)

            xs = np.concatenate([x1, x3[1:], x2[1:]])
            ys = np.concatenate([y1, y3[1:], y2[1:]])
            return np.column_stack([xs, ys]), length

        else:
            # 兩弧一直線
            center1_arc, P1, P2, center2_arc, center1, center2, length, path_type = res
            arc1_len = abs(R * (center1_arc[0] - center1_arc[-1]))
            arc2_len = abs(R * (center2_arc[0] - center2_arc[-1]))
            straight_len = np.linalg.norm(np.array(P2) - np.array(P1))

            arc1_theta = np.linspace(center1_arc[0], center1_arc[-1], max(2, int(arc1_len / distance_resolution)))
            arc2_theta = np.linspace(center2_arc[0], center2_arc[-1], max(2, int(arc2_len / distance_resolution)))

            x1 = center1[0] + R * np.cos(arc1_theta)
            y1 = center1[1] + R * np.sin(arc1_theta)
            x3 = center2[0] + R * np.cos(arc2_theta)
            y3 = center2[1] + R * np.sin(arc2_theta)

            x2 = np.linspace(P1[0], P2[0], max(2, int(straight_len / distance_resolution)))
            y2 = np.linspace(P1[1], P2[1], max(2, int(straight_len / distance_resolution)))

            xs = np.concatenate([x1, x2[1:], x3[1:]])
            ys = np.concatenate([y1, y2[1:], y3[1:]])
            return np.column_stack([xs, ys]), length

    dubins_path, total_length = _build_path_from_result(dubins_result)

    # --- 6) 加上流場漂移（用路徑弧長近似時間） ---
    steps = np.arange(len(dubins_path))
    # 每一步的距離約 = distance_resolution → 時間約 = 距離 / wg_eff
    dt_per_step = distance_resolution / max(wg_eff, 1e-6)
    drift_x = steps * dt_per_step * v_current[0]
    drift_y = steps * dt_per_step * v_current[1]

    current_path = dubins_path.copy()
    current_path[:, 0] += drift_x
    current_path[:, 1] += drift_y

    return dubins_path, current_path
