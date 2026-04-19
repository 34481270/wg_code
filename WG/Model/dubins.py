import sys
project_root = r"C:\Users\david ho\Desktop\WG"
if project_root not in sys.path:
    sys.path.insert(0, project_root)                            
import numpy as np
from WG.Model import commonTangent # type: ignore
import matplotlib.pyplot as plt


def dubins(start, goal, R):
    if np.arctan2(goal[1] - start[1], goal[0] - start[0]) == start[2] and start[2] == goal[2]:
        return 1, start[:2], goal[:2], 1, 1, 1, np.linalg.norm(np.array(goal[:2]) - np.array(start[:2])), 'S', 1, 1
    
    data = []
    
    data.append(RSR(start, goal, R))
    # data.append(LSL(start, goal, R))
    data.append(LSR(start, goal, R))
    data.append(RSL(start, goal, R))
    
    center1 = start[:2] + R * np.array([-np.sin(start[2]), np.cos(start[2])])
    center2 = goal[:2] + R * np.array([-np.sin(goal[2]), np.cos(goal[2])])
    center3 = start[:2] + R * np.array([np.sin(start[2]), -np.cos(start[2])])
    center4 = goal[:2] + R * np.array([np.sin(goal[2]), -np.cos(goal[2])])
    if np.linalg.norm(center1 - center2) < 4 * R:
        data.append(LRL(start, goal, R))
    if np.linalg.norm(center3 - center4) < 4 * R:
        data.append(RLR(start, goal, R))
    best_path = min(data, key=lambda x: x[6])
    
    return best_path

def RSR(start, goal, R):

    path_type = 'RSR'
    center1 = np.array([start[0] + R * np.sin(start[2]), start[1] - R * np.cos(start[2])])
    center2 = np.array([goal[0] + R * np.sin(goal[2]), goal[1] - R * np.cos(goal[2])])
    # Compute outer tangents
    
    P1, P2 = commonTangent.outerTangent(center1, center2, R, R, 'R', 'R', path_type)
    
    # Compute start arc
    theta1 = np.arctan2(P1[1] - center1[1], P1[0] - center1[0])
    theta1_2 = np.arctan2(start[1] - center1[1], start[0] - center1[0])
    if theta1 < theta1_2:
        theta1 += 2 * np.pi
    center1_arc = np.linspace(theta1_2, theta1 - 2 * np.pi, num=100)
    
    # Compute goal arc
    theta2 = np.arctan2(P2[1] - center2[1], P2[0] - center2[0])
    theta2_2 = np.arctan2(goal[1] - center2[1], goal[0] - center2[0])
    if theta2_2 < theta2:
        theta2_2 += 2 * np.pi
    center2_arc = np.linspace(theta2, theta2_2 - 2 * np.pi, num=100)
    
    # Compute path length
    arc1 = np.abs(center1_arc[0] - center1_arc[-1]) * R
    straight = np.linalg.norm(P1 - P2)
    arc2 = np.abs(center2_arc[0] - center2_arc[-1]) * R
    length = arc1 + straight + arc2
    
    return center1_arc, P1, P2, center2_arc, center1, center2, length, path_type

def LSR(start, goal, R):

    path_type = 'LSR'
    center1 = np.array([start[0] - R * np.sin(start[2]), start[1] + R * np.cos(start[2])])
    center2 = np.array([goal[0] + R * np.sin(goal[2]), goal[1] - R * np.cos(goal[2])])
    # Compute inner tangents
    P1, P2 = commonTangent.innerTangent(center1, center2, R, 'L', 'R',path_type)
    
    # Compute start arc
    theta1 = np.arctan2(P1[1] - center1[1], P1[0] - center1[0])
    theta1_2 = np.arctan2(start[1] - center1[1], start[0] - center1[0])
    if theta1 < theta1_2:
        theta1 += 2 * np.pi
    center1_arc = np.linspace(theta1_2, theta1, num=100)
    
    # Compute goal arc
    theta2 = np.arctan2(P2[1] - center2[1], P2[0] - center2[0])
    theta2_2 = np.arctan2(goal[1] - center2[1], goal[0] - center2[0])
    if theta2_2 < theta2:
        theta2_2 += 2 * np.pi
    center2_arc = np.linspace(theta2, theta2_2 - 2 * np.pi, num=100)
    
    # Compute path length
    arc1 = np.abs(center1_arc[0] - center1_arc[-1]) * R
    straight = np.linalg.norm(P1 - P2)
    arc2 = np.abs(center2_arc[0] - center2_arc[-1]) * R
    length = arc1 + straight + arc2
    
    return center1_arc, P1, P2, center2_arc, center1, center2, length, path_type


def RSL(start, goal, R):
    
    path_type = 'RSL'
    center1 = np.array([start[0] + R * np.sin(start[2]), start[1] - R * np.cos(start[2])])
    center2 = np.array([goal[0] - R * np.sin(goal[2]), goal[1] + R * np.cos(goal[2])])
    P1, P2 = commonTangent.innerTangent(center1, center2, R, 'R', 'L',path_type)
    
    theta1 = np.arctan2(P1[1] - center1[1], P1[0] - center1[0])
    theta1_2 = np.arctan2(start[1] - center1[1], start[0] - center1[0])
    if theta1 < theta1_2:
        theta1 += 2 * np.pi
    center1_arc = np.linspace(theta1_2, theta1 - 2 * np.pi, num=100)
    
    theta2 = np.arctan2(P2[1] - center2[1], P2[0] - center2[0])
    theta2_2 = np.arctan2(goal[1] - center2[1], goal[0] - center2[0])
    if theta2_2 < theta2:
        theta2_2 += 2 * np.pi
    center2_arc = np.linspace(theta2, theta2_2, num=100)
    
    arc1 = np.abs(center1_arc[0] - center1_arc[-1]) * R
    straight = np.linalg.norm(P1 - P2)
    arc2 = np.abs(center2_arc[0] - center2_arc[-1]) * R
    length = arc1 + straight + arc2
    
    return center1_arc, P1, P2, center2_arc, center1, center2, length, path_type

def LSL(start, goal, R):

    path_type = 'LSL'
    center1 = np.array([start[0] - R * np.sin(start[2]), start[1] + R * np.cos(start[2])])
    center2 = np.array([goal[0] - R * np.sin(goal[2]), goal[1] + R * np.cos(goal[2])])
    P1, P2 = commonTangent.outerTangent(center1, center2, R, R, 'L', 'L',path_type)
    
    theta1 = np.arctan2(P1[1] - center1[1], P1[0] - center1[0])
    theta1_2 = np.arctan2(start[1] - center1[1], start[0] - center1[0])
    if theta1 < theta1_2:
        theta1 += 2 * np.pi
    center1_arc = np.linspace(theta1_2, theta1, num=100)
    
    theta2 = np.arctan2(P2[1] - center2[1], P2[0] - center2[0])
    theta2_2 = np.arctan2(goal[1] - center2[1], goal[0] - center2[0])
    if theta2_2 < theta2:
        theta2_2 += 2 * np.pi
    center2_arc = np.linspace(theta2, theta2_2, num=100)
    
    arc1 = np.abs(center1_arc[0] - center1_arc[-1]) * R
    straight = np.linalg.norm(P1 - P2)
    arc2 = np.abs(center2_arc[0] - center2_arc[-1]) * R
    length = arc1 + straight + arc2
    
    return center1_arc, P1, P2, center2_arc, center1, center2, length, path_type

def isosceles_vertex(P1, P2, a):
    """
    Compute the vertex and midpoints of an isosceles triangle.
    """
    M = (P1 + P2) / 2
    b = np.linalg.norm(P2 - P1)
    
    if 2 * a <= b:
        raise ValueError("Invalid isosceles triangle, check side lengths.")
    
    dir_vector = (P2 - P1) / b
    normal_vector = np.array([-dir_vector[1], dir_vector[0]])
    h = np.sqrt(a**2 - (b / 2)**2)
    
    P3a = M + h * normal_vector
    P3b = M - h * normal_vector
    
    M1a = (P1 + P3a) / 2
    M2a = (P2 + P3a) / 2
    M1b = (P1 + P3b) / 2
    M2b = (P2 + P3b) / 2
    
    return P3a, P3b, M1a, M2a, M1b, M2b

def RLR(start, goal, R):

    path_type = 'RLR'
    center1 = np.array([start[0] + R * np.sin(start[2]), start[1] - R * np.cos(start[2])])
    center2 = np.array([goal[0] + R * np.sin(goal[2]), goal[1] - R * np.cos(goal[2])])
    
    P3a, P3b, M1a, M2a, M1b, M2b = isosceles_vertex(center1, center2, R * 2)
    
    theta1 = np.arctan2(M1a[1] - center1[1], M1a[0] - center1[0])
    theta1_2 = np.arctan2(start[1] - center1[1], start[0] - center1[0])
    if theta1 < theta1_2:
        theta1 += 2 * np.pi
    center1_arc = np.linspace(theta1_2, theta1 - 2 * np.pi, num=100)
    
    theta2 = np.arctan2(M2a[1] - center2[1], M2a[0] - center2[0])
    theta2_2 = np.arctan2(goal[1] - center2[1], goal[0] - center2[0])
    if theta2_2 < theta2:
        theta2_2 += 2 * np.pi
    center2_arc = np.linspace(theta2, theta2_2 - 2 * np.pi, num=100)
    
    theta3 = np.arctan2(M1a[1] - P3a[1], M1a[0] - P3a[0])
    theta3_2 = np.arctan2(M2a[1] - P3a[1], M2a[0] - P3a[0])
    if theta3 > theta3_2:
        theta3_2 += 2 * np.pi
    center3 = P3a
    center3_arc = np.linspace(theta3_2, theta3, num=100)
    
    arc1 = np.abs(center1_arc[0] - center1_arc[-1]) * R
    arc2 = np.abs(center2_arc[0] - center2_arc[-1]) * R
    arc3 = np.abs(center3_arc[0] - center3_arc[-1]) * R
    length = arc1 + arc2 + arc3
    
    return center1_arc, M1a, M2a, center2_arc, center1, center2, length, path_type, center3, center3_arc

def LRL(start, goal, R):

    path_type = 'LRL'
    center1 = np.array([start[0] - R * np.sin(start[2]), start[1] + R * np.cos(start[2])])
    center2 = np.array([goal[0] - R * np.sin(goal[2]), goal[1] + R * np.cos(goal[2])])
    
    P3a, P3b, M1a, M2a, M1b, M2b = isosceles_vertex(center1, center2, R * 2)
    
    theta1 = np.arctan2(M1a[1] - center1[1], M1a[0] - center1[0])
    theta1_2 = np.arctan2(start[1] - center1[1], start[0] - center1[0])
    if theta1 < theta1_2:
        theta1 += 2 * np.pi
    center1_arc = np.linspace(theta1_2, theta1, num=100)
    
    theta2 = np.arctan2(M2a[1] - center2[1], M2a[0] - center2[0])
    theta2_2 = np.arctan2(goal[1] - center2[1], goal[0] - center2[0])
    if theta2_2 < theta2:
        theta2_2 += 2 * np.pi
    center2_arc = np.linspace(theta2, theta2_2, num=100)
    
    theta3 = np.arctan2(M1a[1] - P3a[1], M1a[0] - P3a[0])
    theta3_2 = np.arctan2(M2a[1] - P3a[1], M2a[0] - P3a[0])
    if theta3 < theta3_2:
        theta3 += 2 * np.pi
    center3 = P3a
    center3_arc = np.linspace(theta3, theta3_2, num=100)
    
    arc1 = np.abs(center1_arc[0] - center1_arc[-1]) * R
    arc2 = np.abs(center2_arc[0] - center2_arc[-1]) * R
    arc3 = np.abs(center3_arc[0] - center3_arc[-1]) * R
    length = arc1 + arc2 + arc3
    
    return center1_arc, M1a, M2a, center2_arc, center1, center2, length, path_type, center3, center3_arc



