# import time
# import serial
# import threading
# from datetime import datetime
# import os

# file_name = 'GPS.txt'
# start_key = False
# end_key = False
# class GPS:
#     def __init__(self):

#         self.file_counter = 1
#         threading.Thread(target = self.GPS_connect).start()
        
        
#     def GPS_connect(self):
        
#         while 1:
#             try:
#                 self.ser =serial.Serial(port = '/dev/tty.usbserial-140', baudrate = 9600, timeout=1)
#                 self.init_create_file()
#                 threading.Thread(target = self.GPS_Data_Analysis).start()
#                 break
#             except serial.SerialException:
#                 print("GPS connect issue, please check the connection")
#                 print("-----------------------------------------------")
#                 time.sleep(1)

#     def init_create_file(self):
#         global file_name
#         while os.path.exists(file_name):
#             file_name = f'GPS_{self.file_counter}.txt'
#             self.file_counter += 1
#         self.file = open(file_name,'w',encoding='utf-8')
#         # 打印檔案名稱以確認
#         print("開啟或創建檔案：" + file_name)
        
        
#     def GPS_Data_Saving(self,GPS_Data):
#         current_time = datetime.now().time()
#         GPS_Data = str(current_time) + "\t" +GPS_Data
#         if self.file:
#             try:
#                 self.file.write(GPS_Data)
#                 self.file.flush()
#             except IOError:
#                 print("writing failed")


#     def is_available(str):
#         result = 0
#         for i in str:
#             for j in i:
#                 result ^= ord(j)
#         result ^= ord(str[-1][0])
#         result ^= ord(str[-1][1])
#         if result == int(str[-1],16):
#             return 1
#         else: 
#             return 0
        
#     def printOut(data):  
#         print("UTC時間"+data[1][0:2]+ ":" + data[1][2:4]+":"+data[1][4:6])
#         if data[2] == 'A':
#             print("定位狀態 有效")
#         else:
#             print("定位狀態 無效")
#         print("緯度 : " + data[3])
#         print("緯度半球 : " + data[4])
#         print("經度" + data[5])
#         print("經度半球 : " + data[6])
#         print("地面速率 : " + data[7])
#         print("地面航向 : " + data[8])
#         print("UTC日期  : 20"+ data[9][4:6]+ "年" + data[9][2:4]+"月"+data[9][0:2]+"日")
#         print("磁偏角    : " + data[10])
#         print("磁偏角方向 : " + data[11])
#         #print("模式 : 自主定位/差分/估算/資料無效" + data[12] + data[13] + data[14] + dataCheck)
#         print("-----------------------------------------------")


#     def calculate_checksum(sentence):
#         checksum = 0
#         for char in sentence:
#             checksum ^= ord(char)
#         return checksum

#     # ls -al /dev/tty.usbserial*
#     # dmesg | grep tty (serial port command for Respbian) (Respbian, which is Respberry OS)
#     def GPS_Data_Analysis(self):
#         global start_key
#         global end_key
#         print("GPS turn on successfully")
#         while 1:
#             try:
#                 data = self.ser.readline()
#                 print(data)
#                 # if raw_data[0] != 36:
#                 #     continue
#                 raw_data = data.decode('utf-8')
#                 data = raw_data[raw_data.find('$')+1 : raw_data.find('*')]
#                 # data = data.replace('*',',').split(',')[-1]
#                 if start_key:
#                     self.GPS_Data_Saving(GPS_Data= "____________Start____________\n")
#                     start_key = False
#                 elif end_key:
#                     self.GPS_Data_Saving(GPS_Data= "_____________END_____________\n")
#                     end_key = False
#                 if data[0:5] == 'GNRMC': #and self.calculate_checksum(data) == int(raw_data,16):
#                     self.GPS_Data_Saving(GPS_Data=raw_data)  
#                     print(raw_data)   
#             except serial.SerialException as e:
#                 print("serialException")
#                 print("Serial communication error:", e)
#                 time.sleep(1)
#                 try:
#                     self.ser = serial.Serial(port = '/dev/tty.usbserial-130', baudrate = 9600, timeout=1)
#                 except serial.SerialException:
#                     None
#             except UnicodeDecodeError:
#                 print("Data decoding error")
#                 time.sleep(1)
#             except ValueError:
#                 print("Checksum calculation or hex conversion error")
#                 time.sleep(1)
#             except Exception as e:
#                 print("An unexpected error occurred:", e)
#                 time.sleep(1)
                
# myGPS = GPS()

# while 1:
#     time.sleep(1)


# server_test.py
import socket
s = socket.socket()
s.bind(("0.0.0.0", 5005))
s.listen(1)
print("Listening...")

conn, addr = s.accept()
print(f"Connected by {addr}")
while True:
    data = conn.recv(1024)
    if not data:
        break
    print("?", data.decode())
