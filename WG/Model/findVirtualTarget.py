import sys
project_root = r"C:\Users\david ho\Desktop\WG"
if project_root not in sys.path:
    sys.path.insert(0, project_root)
import numpy as np
import matplotlib.pyplot as plt
from WG.Model import dubins # type: ignore

def wrap_to_180(a): 
    return (a + 180) % 360 - 180

def findVirtualTarget(start, goal, R, current_speed, wg_speed, current_angle, distance_resolution= 0.1):
    # 初始設定

    wg_speed = max(wg_speed, 0.5)
    t_upper = 36000
    t_lower = 0
    t_mid = 18000

    # # 修正速度（考慮水流與前進方向） 
    # tmp = wg_speed * np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))]) \
    #       - current_speed * np.array([np.cos(np.radians(current_angle)), np.sin(np.radians(current_angle))])
    # wg_speed_new = np.linalg.norm(tmp)
    # wg_speed = max(wg_speed_new, 0.01)
    yaw = start[2]
    print('yaw = ',yaw)
    vh = np.array([np.cos(yaw), np.sin(yaw)])
    print('vh = ',vh)
    print('vh = ',[np.cos(yaw), np.sin(yaw)])

    # v_ground  = wg_speed * vh
    # v_current = current_speed * np.array([np.cos(np.radians(current_angle)),
    #                                     np.sin(np.radians(current_angle))])
    # v_thr = v_ground - v_current
    # wg_speed = max(np.linalg.norm(v_thr), 0.05)
    print("============================")
    print('wg對地速度 = ', wg_speed)
    print('  海流速度 = ', current_speed)
    print("============================")  
    
    # 二分法找交會時間
    if wg_speed < current_speed:
        wg_speed = current_speed + 0.5
    current_angle = (90 - current_angle)%360
    counter = 0
    while 1:
        counter += 1
        virtual_target = goal[:2] - current_speed * np.array([np.cos(np.radians(current_angle)), np.sin(np.radians(current_angle))]) * t_mid
        dubins_result = dubins.dubins(start, np.append(virtual_target, goal[2]), R)
        total_length = dubins_result[6]
        t_wg = total_length / wg_speed
        
        if abs(t_wg - t_mid) < 5:
            break
        elif t_mid > t_wg:
            t_upper = t_mid
        else:
            t_lower = t_mid
        t_mid = (t_upper + t_lower) / 2
        if counter >= 40:
            return None, None
        
        print(t_mid)
    # 製作 dubins path（以距離解析度切分）
    if len(dubins_result) == 10:
        center1_arc, M1a, M2a, center2_arc, center1, center2, length, path_type, center3, center3_arc = dubins_result
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

        dubins_path = np.column_stack([np.concatenate([x1, x3[1:], x2[1:]]),
                                       np.concatenate([y1, y3[1:], y2[1:]])])
    else:
        center1_arc, P1, P2, center2_arc, center1, center2, length, path_type = dubins_result
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

        dubins_path = np.column_stack([np.concatenate([x1, x2[1:], x3[1:]]),
                                       np.concatenate([y1, y2[1:], y3[1:]])])

    # 流場影響模擬（用每個點到起點的距離近似時間偏移）
    steps = np.arange(len(dubins_path))
    drift_x = steps * distance_resolution * (current_speed / wg_speed) * np.cos(np.radians(current_angle))
    drift_y = steps * distance_resolution * (current_speed / wg_speed) * np.sin(np.radians(current_angle))

    current_path = dubins_path.copy()
    current_path[:, 0] += drift_x
    current_path[:, 1] += drift_y
    

    return dubins_path, current_path

