import numpy as np
import math
class PID:
    def __init__(self, Kp=0.6, Ki=0.03, Kd=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.error_integral = 0
        self.error_prev = 0
        self.rudder_angle = 0

    def rudder_pid_parameter_adjust(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        print(self.Kp)
        print(self.Ki)
        print(self.Kd)
        print('rudder PID參數設定完成')

    def setZero(self):
        self.error_integral = 0
        self.error_prev = 0
        self.rudder_angle = 0

    def wrap_to_180(self, angle_deg):
        return (angle_deg + 180) % 360 - 180


    def compute(self, course_angle, r_cmd, dt):
        # 1) normalize angles
        course_angle = (course_angle + 360) % 360
        r_cmd = (r_cmd + 360) % 360
        error = self.wrap_to_180(course_angle - r_cmd)

        # 2) cap dt，避免一次積太多
        dt = max(1e-3, min(dt, 0.2))

        # 3) PD 先算
        de = (error - self.error_prev) / dt
        P = self.Kp * error
        D = self.Kd * de

        # 4) 嘗試更新積分
        i_candidate = self.error_integral + error * dt

        # 把 I 項夾到不會讓輸出超出限制（anti-windup clamping）
        u_min, u_max = -30.0, 30.0
        if self.Ki > 0:
            i_min = (u_min - P - D) / self.Ki
            i_max = (u_max - P - D) / self.Ki
            i_candidate = max(min(i_candidate, i_max), i_min)

        I = self.Ki * i_candidate
        u = P + I + D

        # 實際輸出限幅
        u = max(min(u, u_max), u_min)

        # 只有在未飽和或誤差在「往回拉」輸出時，才接受新的 integral（條件積分）
        if (u > u_min and u < u_max) or (u == u_max and error < 0) or (u == u_min and error > 0):
            self.error_integral = i_candidate
        # 否則不更新 self.error_integral，防止風up

        self.error_prev = error
        self.rudder_angle = -u  # 你原本是取負
        
        return self.rudder_angle
