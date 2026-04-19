import PID

pid = PID.PID()
print(pid.compute(50, 20, 0.5))
# compute(self, course_angle, r_cmd, dt):


