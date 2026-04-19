# WG/sensor/GPS_IMU_MOCK.py
import threading
import time
import random

class GPS(threading.Thread):
    def __init__(self, shared):
        super().__init__()
        self.shared = shared
        self._stop_event = threading.Event()
        self.lat = 25.15
        self.lon = 121.75

    def run(self):
        print("🛰️ 模擬 GPS 啟動")
        while not self._stop_event.is_set():
            self.lat += 0.00003
            self.lon += 0.00003
            speed = 5.0 + random.uniform(-0.2, 0.2)
            heading = 60
            self.shared['GPS']['lat'] = self.lat
            self.shared['GPS']['lon'] = self.lon
            self.shared['GPS']['speed'] = speed
            self.shared['GPS']['heading'] = heading
            self.shared['IMU_Glider']['angle_yaw'] = heading

            time.sleep(1.0)

    def stop(self):
        self._stop_event.set()


class IMU(threading.Thread):
    def __init__(self, shared):
        super().__init__()
        self.shared = shared
        self._stop_event = threading.Event()

    def run(self):
        print("📦 模擬 IMU 啟動")
        while not self._stop_event.is_set():
            self.shared['IMU']['yaw'] = 30
            self.shared['IMU']['speed'] = 1.0 + random.uniform(-0.1, 0.1)
            time.sleep(1.0)

    def stop(self):
        self._stop_event.set()