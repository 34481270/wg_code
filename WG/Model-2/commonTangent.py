import numpy as np
import matplotlib.pyplot as plt

def outerTangent(circle1, circle2, r1, r2, typeA, typeB,path_type):
    # print('path_type = ', path_type)
    # print('circle1 = ',circle1)
    # print('circle2 = ', circle2)
    # print('r1 = ',r1)
    # print('r2 = ',r2)
    a1, b1 = circle1
    a2, b2 = circle2
    
    a21, b21 = a2 - a1, b2 - b1
    d2 = a21**2 + b21**2
    
    if d2 < (r2 - r1)**2:
        raise ValueError("No solution possible")
    
    r21 = (r2 - r1) / d2
    
    s21 = np.sqrt(d2 - (r2 - r1)**2) / d2
    
    u1 = np.array([-a21 * r21 - b21 * s21, -b21 * r21 + a21 * s21])  # Left unit vector
    u2 = np.array([-a21 * r21 + b21 * s21, -b21 * r21 - a21 * s21])  # Right unit vector
    
    L1, L2 = np.array([a1, b1]) + r1 * u1, np.array([a2, b2]) + r2 * u1  # Left line tangency points
    R1, R2 = np.array([a1, b1]) + r1 * u2, np.array([a2, b2]) + r2 * u2  # Right line tangency points
    
    if typeA == 'R' and typeB == 'R':
        return L1, L2
    else:
        return R1, R2

def innerTangent(start, goal, r, typeA, typeB,path_type):
    # print('path_type = ', path_type)
    # print('circle1 = ',circle1)
    # print('circle2 = ', circle2)
    # print('r1 = ',r1)
    # print('r2 = ',r2)
    D = np.linalg.norm(goal - start)
    s_to_g = np.degrees(np.arctan2(goal[1] - start[1], goal[0] - start[0]))
    g_to_s = np.degrees(np.arctan2(start[1] - goal[1], start[0] - goal[0]))
    
    angle = np.degrees(np.arccos(r / (D / 2)))
    start_1, start_2 = s_to_g + angle, s_to_g - angle
    goal_1, goal_2 = g_to_s + angle, g_to_s - angle
    
    point2 = np.array([start[0] + r * np.cos(np.radians(start_1)), start[1] + r * np.sin(np.radians(start_1))])
    point1 = np.array([start[0] + r * np.cos(np.radians(start_2)), start[1] + r * np.sin(np.radians(start_2))])
    point4 = np.array([goal[0] + r * np.cos(np.radians(goal_1)), goal[1] + r * np.sin(np.radians(goal_1))])
    point3 = np.array([goal[0] + r * np.cos(np.radians(goal_2)), goal[1] + r * np.sin(np.radians(goal_2))])
    
    if typeA == 'R' and typeB == 'L':
        return point2, point4
    else:
        return point1, point3

# start = np.array([0, 0, 0])
# goal = np.array([10, 0, 2])
# R = 2

# P1,P2 = inner_tangent(start,goal,R,'R','R');
# print(P1)
# print(P2)

# ig, ax = plt.subplots()
# ax.set_aspect('equal')

# # 畫圓
# circle = plt.Circle(start, R, color='b', fill=False, linewidth=2)
# circle1 = plt.Circle(goal, R, color='b', fill=False, linewidth=2)
# ax.add_patch(circle)
# ax.add_patch(circle1)

# # 畫線
# ax.plot([P1[0], P2[0]], [P1[1], P2[1]], 'r-', linewidth=2)

# # 畫點
# ax.plot(P1[0], P1[1], 'go', markersize=8)  # 綠色圓點
# ax.plot(P2[0], P2[1], 'go', markersize=8)

# plt.grid()
# plt.show()