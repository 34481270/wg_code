# sensor_module.py
import serial
import time
import threading
import socket
import os
from datetime import datetime
from WG.sensor import device_model as deviceModel  # type: ignore
from WG.sensor.jy901s_dataProcessor import JY901SDataProcessor  # type: ignore
from WG.sensor.wit_protocol_resolver import WitProtocolResolver  # type: ignore

# ls -al /dev/tty.usbserial*
class GPS(threading.Thread):
    def __init__(self, shared, port='/dev/tty.usbserial-120', baudrate=9600):
        super().__init__()
        self.shared = shared
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.file = None
        self.file_counter = 1
        self.file_name = 'GPS.txt'
        self._stop_event = threading.Event()

    def run(self):
        self.GPS_connect()

    def stop(self):
        self._stop_event.set()
        if self.file:
            self.file.close()

    def GPS_connect(self):
        while not self._stop_event.is_set():            
            try:
                self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=1)
                self.init_create_file()
                threading.Thread(target=self.GPS_Data_Analysis).start()
                break
            except serial.SerialException:
                print("GPS connect issue, please check the connection")
                time.sleep(1)

    def init_create_file(self):
        while os.path.exists(self.file_name):
            self.file_name = f'GPS_{self.file_counter}.txt'
            self.file_counter += 1
        self.file = open(self.file_name, 'w', encoding='utf-8')
        print("GPS file:", self.file_name)

    def GPS_Data_Saving(self, GPS_Data):
        current_time = datetime.now().time()
        GPS_Data = str(current_time) + "\t" + GPS_Data
        if self.file:
            try:
                self.file.write(GPS_Data)
                self.file.flush()
            except IOError:
                print("writing failed")

    def GPS_Data_Analysis(self):
        print("GPS turned on successfully")
        while not self._stop_event.is_set():
            try:
                data = self.ser.readline()
                raw_data = data.decode('utf-8')
                data = raw_data[raw_data.find('$')+1: raw_data.find('*')]
                if data[0:5] == 'GNRMC':
                    print('1')
                    self.GPS_Data_Saving(GPS_Data=raw_data)
                    parts = raw_data.split(',')
                    if len(parts) > 9:
                        try: # 1.時間 2.狀態 3.緯度 4.經度 5.速度 6.航向 7.日期 8.磁偏角 9.CheckSum
   
                            self.shared['GPS']['lat'] = float(parts[3])
                            self.shared['GPS']['lon'] = float(parts[4])
                            self.shared['GPS']['speed'] = float(parts[5])
                            self.shared['GPS']['heading'] = float(parts[6])
                        except:
                            pass
            except Exception as e:
                print("GPS Error:", e)
                time.sleep(1)


class IMU(threading.Thread):
    def __init__(self, shared):
        super().__init__()
        self.device = deviceModel.DeviceModel(
            "myIMU",
            WitProtocolResolver(),
            JY901SDataProcessor(),
            "51_0",
            '/dev/cu.usbserial-1120'
        )
        self._writeF = None
        self._IsWriteF = False
        self.shared = shared
        self._stop_event = threading.Event()

    def run(self):
        self.device.openDevice()
        self.read_config()
        self.device.dataProcessor.onVarChanged.append(self.on_update)
        self.start_record()
        while not self._stop_event.is_set():
            time.sleep(0.1)
        self.end_record()
        self.device.closeDevice()

    def stop(self):
        self._stop_event.set()

    def read_config(self):
        tVals = self.device.readReg(0x02, 3)
        if len(tVals) > 0:
            print("返回结果：" + str(tVals))
        else:
            print("无返回")
        tVals = self.device.readReg(0x23, 2)
        if len(tVals) > 0:
            print("返回结果：" + str(tVals))
        else:
            print("无返回")

    def set_config(self):
        self.device.unlock()
        time.sleep(0.1)
        self.device.writeReg(0x03, 6)
        time.sleep(0.1)
        self.device.writeReg(0x23, 0)
        time.sleep(0.1)
        self.device.writeReg(0x24, 0)
        time.sleep(0.1)
        self.device.save()

    def acceleration_calibration(self):
        self.device.AccelerationCalibration()
        print("加计校准结束")

    def filed_calibration(self):
        self.device.BeginFiledCalibration()
        if input("请分别绕XYZ轴慢速转动一圈，三轴转圈完成后，结束校准（Y/N)？").lower() == "y":
            self.device.EndFiledCalibration()
            print("结束磁场校准")

    def on_update(self, device_model):
        print("芯片时间:" + str(device_model.getDeviceData("Chiptime"))
            #   , " 温度:" + str(device_model.getDeviceData("temperature"))
              , " 加速度：" + str(device_model.getDeviceData("accX")) + "," + str(device_model.getDeviceData("accY")) + "," + str(device_model.getDeviceData("accZ"))
            #   , " 角速度:" + str(device_model.getDeviceData("gyroX")) + "," + str(device_model.getDeviceData("gyroY")) + "," + str(device_model.getDeviceData("gyroZ"))
            #   , " 角度:" + str(device_model.getDeviceData("angleX")) + "," + str(device_model.getDeviceData("angleY")) + "," + str(device_model.getDeviceData("angleZ"))
            #   , " 磁场:" + str(device_model.getDeviceData("magX")) + "," + str(device_model.getDeviceData("magY")) + "," + str(device_model.getDeviceData("magZ"))
            #   , " 经度:" + str(device_model.getDeviceData("lon")) + " 纬度:" + str(device_model.getDeviceData("lat"))
            #   , " 航向角:" + str(device_model.getDeviceData("Yaw")) + " 地速:" + str(device_model.getDeviceData("Speed"))
            #   , " 四元素:" + str(device_model.getDeviceData("q1")) + "," + str(device_model.getDeviceData("q2")) + "," + str(device_model.getDeviceData("q3")) + "," + str(device_model.getDeviceData("q4"))
              )

        if self._IsWriteF:
            Tempstr = " " + str(device_model.getDeviceData("Chiptime"))
            Tempstr += "\t" + str(device_model.getDeviceData("accX")) + "\t" + str(device_model.getDeviceData("accY")) + "\t" + str(device_model.getDeviceData("accZ"))
            Tempstr += "\t" + str(device_model.getDeviceData("gyroX")) + "\t" + str(device_model.getDeviceData("gyroY")) + "\t" + str(device_model.getDeviceData("gyroZ"))
            Tempstr += "\t" + str(device_model.getDeviceData("angleX")) + "\t" + str(device_model.getDeviceData("angleY")) + "\t" + str(device_model.getDeviceData("angleZ"))
            Tempstr += "\t" + str(device_model.getDeviceData("temperature"))
            Tempstr += "\t" + str(device_model.getDeviceData("magX")) + "\t" + str(device_model.getDeviceData("magY")) + "\t" + str(device_model.getDeviceData("magZ"))
            Tempstr += "\t" + str(device_model.getDeviceData("lon")) + "\t" + str(device_model.getDeviceData("lat"))
            Tempstr += "\t" + str(device_model.getDeviceData("Yaw")) + "\t" + str(device_model.getDeviceData("Speed"))
            Tempstr += "\t" + str(device_model.getDeviceData("q1")) + "\t" + str(device_model.getDeviceData("q2"))
            Tempstr += "\t" + str(device_model.getDeviceData("q3")) + "\t" + str(device_model.getDeviceData("q4"))
            Tempstr += "\r\n"
            Tempstr = str(datetime.now().time()) + "\t" + Tempstr
            self._writeF.write(Tempstr)

            self.shared['IMU']['angularVelocity_yaw'] = device_model.getDeviceData("gyroZ")
            self.shared['IMU']['angle_yaw'] = device_model.getDeviceData("angleZ")




    def start_record(self):
        self._writeF = open(datetime.now().strftime('%Y%m%d%H%M%S') + ".txt", "w")
        self._IsWriteF = True
        Tempstr = "Chiptime"
        Tempstr += "\tax(g)\tay(g)\taz(g)"
        Tempstr += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
        Tempstr += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
        Tempstr += "\tT(°)"
        Tempstr += "\tmagx\tmagy\tmagz"
        Tempstr += "\tlon\tlat"
        Tempstr += "\tYaw\tSpeed"
        Tempstr += "\tq1\tq2\tq3\tq4"
        Tempstr += "\r\n"
        Tempstr = "World Time    " + Tempstr
        self._writeF.write(Tempstr)
        
        print("开始记录数据")

    def end_record(self):
        self._IsWriteF = False
        if self._writeF:
            self._writeF.close()
        print("结束记录数据")



class IMU_Glider(threading.Thread):
    def __init__(self, shared):
        super().__init__()
        self.device = deviceModel.DeviceModel(
            "IMU_button",
            WitProtocolResolver(),
            JY901SDataProcessor(),
            "51_0",
            '/dev/WG_IMU_button'
        )
        self._writeF = None
        self._IsWriteF = False
        self.shared = shared
        self._stop_event = threading.Event()

    def run(self):
        self.device.openDevice()
        self.read_config()
        self.device.dataProcessor.onVarChanged.append(self.on_update)
        self.start_record()
        while not self._stop_event.is_set():
            time.sleep(0.1)
        self.end_record()
        self.device.closeDevice()

    def stop(self):
        self._stop_event.set()

    def read_config(self):
        tVals = self.device.readReg(0x02, 3)
        if len(tVals) > 0:
            print("返回结果：" + str(tVals))
        else:
            print("无返回")
        tVals = self.device.readReg(0x23, 2)
        if len(tVals) > 0:
            print("返回结果：" + str(tVals))
        else:
            print("无返回")

    def set_config(self):
        self.device.unlock()
        time.sleep(0.1)
        self.device.writeReg(0x03, 6)
        time.sleep(0.1)
        self.device.writeReg(0x23, 0)
        time.sleep(0.1)
        self.device.writeReg(0x24, 0)
        time.sleep(0.1)
        self.device.save()

    def acceleration_calibration(self):
        self.device.AccelerationCalibration()
        print("加计校准结束")

    def filed_calibration(self):
        self.device.BeginFiledCalibration()
        if input("请分别绕XYZ轴慢速转动一圈，三轴转圈完成后，结束校准（Y/N)？").lower() == "y":
            self.device.EndFiledCalibration()
            print("结束磁场校准")

    def on_update(self, device_model):
        # print("芯片时间:" + str(device_model.getDeviceData("Chiptime"))
        #       , " 温度:" + str(device_model.getDeviceData("temperature"))
        #       , " 加速度：" + str(device_model.getDeviceData("accX")) + "," + str(device_model.getDeviceData("accY")) + "," + str(device_model.getDeviceData("accZ"))
        #       , " 角速度:" + str(device_model.getDeviceData("gyroX")) + "," + str(device_model.getDeviceData("gyroY")) + "," + str(device_model.getDeviceData("gyroZ"))
        #       , " 角度:" + str(device_model.getDeviceData("angleX")) + "," + str(device_model.getDeviceData("angleY")) + "," + str(device_model.getDeviceData("angleZ"))
        #       , " 磁场:" + str(device_model.getDeviceData("magX")) + "," + str(device_model.getDeviceData("magY")) + "," + str(device_model.getDeviceData("magZ"))
        #       , " 经度:" + str(device_model.getDeviceData("lon")) + " 纬度:" + str(device_model.getDeviceData("lat"))
        #       , " 航向角:" + str(device_model.getDeviceData("Yaw")) + " 地速:" + str(device_model.getDeviceData("Speed"))
        #       , " 四元素:" + str(device_model.getDeviceData("q1")) + "," + str(device_model.getDeviceData("q2")) + "," + str(device_model.getDeviceData("q3")) + "," + str(device_model.getDeviceData("q4"))
        #       )

        if self._IsWriteF:
            Tempstr = " " + str(device_model.getDeviceData("Chiptime"))
            Tempstr += "\t" + str(device_model.getDeviceData("accX")) + "\t" + str(device_model.getDeviceData("accY")) + "\t" + str(device_model.getDeviceData("accZ"))
            Tempstr += "\t" + str(device_model.getDeviceData("gyroX")) + "\t" + str(device_model.getDeviceData("gyroY")) + "\t" + str(device_model.getDeviceData("gyroZ"))
            Tempstr += "\t" + str(device_model.getDeviceData("angleX")) + "\t" + str(device_model.getDeviceData("angleY")) + "\t" + str(device_model.getDeviceData("angleZ"))
            Tempstr += "\t" + str(device_model.getDeviceData("temperature"))
            Tempstr += "\t" + str(device_model.getDeviceData("magX")) + "\t" + str(device_model.getDeviceData("magY")) + "\t" + str(device_model.getDeviceData("magZ"))
            Tempstr += "\t" + str(device_model.getDeviceData("lon")) + "\t" + str(device_model.getDeviceData("lat"))
            Tempstr += "\t" + str(device_model.getDeviceData("Yaw")) + "\t" + str(device_model.getDeviceData("Speed"))
            Tempstr += "\t" + str(device_model.getDeviceData("q1")) + "\t" + str(device_model.getDeviceData("q2"))
            Tempstr += "\t" + str(device_model.getDeviceData("q3" )) + "\t" + str(device_model.getDeviceData("q4"))
            Tempstr += "\r\n"
            Tempstr = str(datetime.now().time()) + "\t" + Tempstr
            self._writeF.write(Tempstr)

            self.shared['IMU_button']['angularVelocity_yaw'] = device_model.getDeviceData("gyroZ")
            self.shared['IMU_button']['angle_yaw'] = device_model.getDeviceData("angleZ")




    def start_record(self):
        filename = "IMU_button_" + datetime.now().strftime('%Y%m%d%H%M%S') + ".txt"
        self._writeF = open(filename, "w")
        self._IsWriteF = True
        Tempstr = "Chiptime"
        Tempstr += "\tax(g)\tay(g)\taz(g)"
        Tempstr += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
        Tempstr += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
        Tempstr += "\tT(°)"
        Tempstr += "\tmagx\tmagy\tmagz"
        Tempstr += "\tlon\tlat"
        Tempstr += "\tYaw\tSpeed"
        Tempstr += "\tq1\tq2\tq3\tq4"
        Tempstr += "\r\n"
        Tempstr = "World Time    " + Tempstr
        self._writeF.write(Tempstr)
        
        print("开始记录数据")

    def end_record(self):
        self._IsWriteF = False
        if self._writeF:
            self._writeF.close()
        print("结束记录数据")







