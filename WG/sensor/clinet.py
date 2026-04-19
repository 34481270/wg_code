import socket
import time
import tkinter as tk


intput_message = ""

# 連接伺服器與除錯
def connect_to_server(host, port):
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((host, port))
            return sock
        except socket.error as e:
            print(f"連接失敗，錯誤：{e}。5秒後重試。")
            time.sleep(5)

# 主程式
def main():
    host = '192.168.1.102'  
    port = 12345
    sock = connect_to_server(host, port)

    while True:
        try:    
            while True:
                message = input("Enter your message ('exit' to close): ")
                sock.sendall(message.encode())
                if message == 'exit': # 輸入exit斷開連結
                    break
                data = sock.recv(1024)
                print(f"Received from server: {data.decode("utf-8")}")
            pass
        except socket.error as e:
            print(f"連接錯誤：{e}，嘗試重新連接。")
            sock.close()
            sock = connect_to_server(host, port)

# 執行主程式
if __name__ == "__main__":
    main()
    # 创建主窗口
    # root = tk.Tk()
    # root.title("Socket Message Sender")

    # # 创建按钮并绑定到发送消息的函数
    # button1 = tk.Button(root, text="Send 'Hello'", command=lambda: on_button_click('Hello'))
    # button1.pack(pady=10)

    # button2 = tk.Button(root, text="Send 'Goodbye'", command=lambda: on_button_click('Goodbye'))
    # button2.pack(pady=10)




