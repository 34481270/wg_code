import numpy as np
import matplotlib.pyplot as plt

# 一條弧形規劃路徑
path = np.array([
    [0, 0],
    [50, 20],
    [100, 60],
    [150, 120],
    [200, 200]
])

# 每段 30 秒的 GPS 位置
GPS = np.array([
    [0, 0],
    [0, -70],
    [-10, -90],
    [-40, -120],
    [-70, -150],
    [-100, -160],
    [-150, -200],
])

delta_t = 30.0

def project_point_to_segment(p, a, b):
    """將點p投影到線段ab上，回傳投影點"""
    ap = p - a
    ab = b - a
    t = np.dot(ap, ab) / np.dot(ab, ab)
    t = np.clip(t, 0, 1)
    return a + t * ab

def find_nearest_projection(p, path):
    """找出path上距離p最近的線段投影點"""
    closest_pt = None
    min_dist = float('inf')
    for i in range(len(path)-1):
        proj = project_point_to_segment(p, path[i], path[i+1])
        d = np.linalg.norm(proj - p)
        if d < min_dist:
            min_dist = d
            closest_pt = proj
    return closest_pt

# 每次loop估測
currents = []
ref_points = []
prev_real = GPS[0]
prev_ref  = find_nearest_projection(prev_real, path) # 找到起始點與路徑重疊的地方
ref_points.append(prev_ref)

for i in range(1, len(GPS)):
    real = GPS[i]
    ref  = find_nearest_projection(real, path)
    ref_points.append(ref)

    real_disp = real - prev_real
    ref_disp  = ref - prev_ref
    v_current = (real_disp - ref_disp) / delta_t
    currents.append(v_current)

    prev_real = real
    prev_ref  = ref

ref_points = np.array(ref_points)
currents = np.array(currents)

# ============ 繪圖 ============

plt.figure(figsize=(7,6))
plt.plot(path[:,0], path[:,1], 'k--', label='Planned Path')
plt.plot(GPS[:,0], GPS[:,1], 'bo-', label='Real GPS')
plt.plot(ref_points[:,0], ref_points[:,1], 'go-', label='Nearest Path Points')

for i in range(1, len(GPS)):
    # 在兩個GPS中間畫流場箭頭
    mid = (GPS[i] + GPS[i-1]) / 2
    plt.arrow(mid[0], mid[1],
              currents[i-1,0]*10, currents[i-1,1]*10,  # *10 是為了放大顯示
              head_width=3, color='r', alpha=0.6, label='Estimated Current' if i==1 else "")



# 印出估測結果
for i, v in enumerate(currents, start=1):
    speed = np.linalg.norm(v)
    angle = np.degrees(np.arctan2(v[1], v[0]))
    # print(f"Segment {i}: estimated current vector = {v} m/s")
    print(f"  - Speed: {speed:.3f} m/s, Angle: {angle:.2f} degrees")
plt.axis('equal')
plt.legend()
plt.title('Current Estimation by GPS vs Path Nearest Points')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.show()