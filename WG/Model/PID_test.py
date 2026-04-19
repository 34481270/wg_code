import sys
project_root = r"C:\Users\david ho\Desktop\WG"
if project_root not in sys.path:
    sys.path.insert(0, project_root)
import numpy as np
import matplotlib.pyplot as plt
import math
from WG.Model import waveGenerator # type: ignore
from WG.Model import KalmanEstimateCurrent # type: ignore
from WG.Model import findHeading # type: ignore
from WG.Model import findVirtualTarget # type: ignore
from scipy.integrate import solve_ivp
from WG.Model import wavegliderDynamic # type: ignore



class PID:
    def __init__(self, Kp=0.4, Ki=0.001, Kd=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.error_integral = 0
        self.error_prev = 0
        self.rudder_angle = 0

    def shortest_angle_diff(self,current, target):
        diff = (target - current + 180) % 360 - 180
        return diff



    def compute(self, course_angle, r_cmd, dt):
        error = self.shortest_angle_diff(course_angle , r_cmd)
        error_derivative = (error - self.error_prev) / dt

        rudder_unsat = self.Kp * error + self.Ki * self.error_integral + self.Kd * error_derivative

        if abs(rudder_unsat) < 30:  
            self.error_integral += error * dt

        self.error_prev = error 

        rudder_angle = self.Kp * error + self.Ki * self.error_integral + self.Kd * error_derivative

        print(f"course_angle: {course_angle}, cmd: {r_cmd}, P: {self.Kp*error:.2f}, I: {self.Ki*self.error_integral:.2f}, D: {self.Kd*error_derivative:.2f}")

        self.rudder_angle = np.clip(-rudder_angle, -30, 30) 
        print(self.rudder_angle)
        return self.rudder_angle



start = np.array([0, 0, 0])
goal = np.array([-90, 0, 0])
glider_state = np.array([
    start[0], start[1], 0, start[2], 0, 0, 0, 0,
    start[0], start[1], 9.8, start[2], 0, 0, 0, 0
])
cmd_angle = math.degrees(np.arctan2(goal[1] - start[1], goal[0] - start[0]))


time = 100
dt = 0.5
counter = 0
t_cal = np.linspace(0, dt, 10000)

pid = PID()
angle = []

simu_angle = []

while counter < time/dt:
    rudder_angle = pid.compute(math.degrees(glider_state[3]), cmd_angle, dt)
    angle.append(rudder_angle)
    
    waveglider_func = lambda t, state: wavegliderDynamic.wavegliderDynamic(t, state, rudder_angle, t_cal, 0, 0, 0)
    sol_with_current = solve_ivp(waveglider_func, [counter * dt, (counter + 1) * dt], glider_state, method='RK45')
    counter = counter + dt
    
    glider_state = sol_with_current.y[:, -1]
    simu_angle.append(sol_with_current.y)

heading_list = [segment[3, -1] for segment in simu_angle]  

t = np.arange(len(heading_list))

heading_deg = np.rad2deg(heading_list)

plt.figure(figsize=(8, 6))

plt.subplot(2, 1, 1)
plt.plot([0, t[-1]], [cmd_angle, cmd_angle], 'r--', label='Command Heading')  
plt.plot(t, heading_deg, 'b-', label='Actual Heading')  
plt.plot(t, -heading_deg, 'b-', label='Actual Heading') 

plt.xlabel('Time Step')
plt.ylabel('Heading (deg)')
plt.title('Wave Glider Heading vs Command')
plt.grid(True)
plt.legend()

# --- 舵角-時間 ---
plt.subplot(2, 1, 2)
plt.plot(t, angle, 'g-', label='Rudder Angle (deg)')
plt.xlabel('Time Step')
plt.ylabel('Rudder Angle (deg)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
