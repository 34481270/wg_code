import serial
import time

# --- 請在此處進行設定 ---
# 請將此處的序列埠名稱，換成您自己電腦上 Arduino 的序列埠名稱
SERIAL_PORT = ' /dev/cu.usbserial-110'  # 範例 (macOS)
# SERIAL_PORT = 'COM3'              # 範例 (Windows)
# SERIAL_PORT = '/dev/ttyACM0'      # 範例 (Linux)

# 請確保此處的鮑率與您 Arduino 程式碼中 Serial.begin() 的設定一致
BAUD_RATE = 9600
# --- 設定結束 ---

# 建立一個序列埠物件
# timeout=1 表示如果一秒內沒有讀到任何資料，程式也不會卡住
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    # 等待一兩秒，確保序列埠穩定
    time.sleep(2) 
    print(f"成功連接到序列埠 {SERIAL_PORT}，鮑率為 {BAUD_RATE}。")
    print("正在等待接收資料... (按下 Ctrl+C 可中斷程式)")

except serial.SerialException as e:
    print(f"錯誤：無法開啟序列埠 {SERIAL_PORT}。")
    print(f"請檢查序列埠名稱是否正確，或是否有其他程式正在使用它。")
    print(f"詳細錯誤訊息：{e}")
    exit() # 如果無法開啟，直接結束程式

try:
    # 進入一個無限迴圈，持續讀取資料
    while True:
        # 檢查序列埠的輸入緩衝區中是否有資料
        if ser.in_waiting > 0:
            # 讀取一行資料，直到遇到換行符 '\n'
            line = ser.readline()
            
            # 將讀取到的位元組(bytes)資料，解碼(decode)成 UTF-8 字串
            # .strip() 會移除字串前後的空白字元和換行符，讓輸出更乾淨
            try:
                decoded_line = line.decode('utf-8').strip()
                print(f"收到資料: {decoded_line}")
            except UnicodeDecodeError:
                print(f"收到無法解碼的資料: {line}")

except KeyboardInterrupt:
    # 當使用者按下 Ctrl+C 時，會觸發這個例外
    print("\n程式被使用者中斷。")

except serial.SerialException as e:
    print(f"序列埠錯誤：{e}")

finally:
    # 無論程式是正常結束還是出現例外，最後都要確保關閉序列埠
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("序列埠已關閉。")