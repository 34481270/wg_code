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
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print('rudder PID參數設定完成')

    def setZero(self):
        self.error_integral = 0
        self.error_prev = 0
        self.rudder_angle = 0

    def wrap_to_180(self, angle_deg):
        return (angle_deg + 180) % 360 - 180


    def compute(self, course_angle, r_cmd, dt):
        course_angle = (course_angle + 360) % 360
        r_cmd = (r_cmd + 360) % 360

        # 2. ????????
        error = self.wrap_to_180(r_cmd - course_angle)

        error_derivative = (error - self.error_prev) / dt
        rudder_unsat = self.Kp * error + self.Ki * self.error_integral + self.Kd * error_derivative

        if abs(rudder_unsat) < 30:
            self.error_integral += error * dt

        self.error_prev = error
        rudder_angle = self.Kp * error + self.Ki * self.error_integral + self.Kd * error_derivative

        # print(self.Kp, self.Ki, self.Kd)
        # print(f"course_angle: {course_angle}, cmd: {r_cmd}, "
            # f"P: {self.Kp*error:.2f}, I: {self.Ki*self.error_integral:.2f}, D: {self.Kd*error_derivative:.2f}")

        self.rudder_angle = np.clip(rudder_angle, -30, 30)
        return self.rudder_angle
