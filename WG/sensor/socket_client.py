import socket
import threading
import time
import json
from queue import Queue, Empty

class SocketSenderThread(threading.Thread):
    def __init__(self,host="localhost", port=5005, retry_interval=2):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.is_UI_ready = 0
        self.retry_interval = retry_interval
        self.socket = None
        self.sendQueue = Queue()   # 送出去的
        self.recvQueue = Queue()   # 收到的

        self._running = True
        
    def connect(self):
        while self._running:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.host, self.port))
                print("✅ 成功連接到地圖伺服器")
                return
            except Exception as e:
                print(f"⚠️ 連線失敗：{e}，{self.retry_interval} 秒後重試...")
                time.sleep(self.retry_interval)
                

    def run(self):
        while self._running:
            self.connect()  
            try:
                while self._running:
                    while not self.sendQueue.empty():
                        try:
                            cmd, data = self.sendQueue.get_nowait()
                            # print("🧾", cmd)  # ✅ 改成只列印真的送出去的
                            json_data = json.dumps({"data": data})
                            message = f"${cmd}@{json_data}*\n"
                            self.send_in_chunks(self.socket, message)
                        except Empty:
                            pass

                    # 收資料
                    try:
                        raw_data = self.socket.recv(4096).decode()
                        if raw_data:
                            cmd, data = self.decode_message(raw_data)
                            if cmd is not None and data is not None:
                                self.recvQueue.put((cmd,data))
                                print(f"收到命令:{cmd} 資料:{data}")
                    except socket.timeout:
                        pass
                    except Exception as e:
                        print(f"❌ 接收錯誤：{e}")
                        break  # 💥 中斷內層 while → 自動重連
            except Exception as e:
                print(f"❌ 傳送錯誤：{e}")
            finally:
                try:
                    self.socket.close()
                except:
                    pass
                print("🔁 將重新嘗試連線...")
                time.sleep(self.retry_interval)



    def send_in_chunks(self, sock, message, chunk_size=8192, delay=0.01):
        """
        將 message 分段傳送到 socket，每段 chunk_size 字元。
        
        Args:
            sock: 一個已連線的 socket 物件
            message (str): 要傳送的完整字串
            chunk_size (int): 每次傳送的最大字元數（預設 1024）
            delay (float): 每段之間的延遲（單位：秒，預設 0.01 秒）
        """
        encoded = message.encode()
        total_len = len(encoded)
        sent = 0

        while sent < total_len:
            chunk = encoded[sent:sent + chunk_size]
            sock.sendall(chunk)
            sent += chunk_size
            if delay > 0:
                time.sleep(delay)


    def decode_message(self, message):
        """
        從自訂協議格式的訊息中解析出 cmd 和 data。
        預期格式：$<cmd>@<json_data>*\\n
        """
        try:
            if not message.startswith("$") or "@" not in message or "*" not in message:
                return None, None
            core = message.strip()[1:]  # 去掉開頭 $
            cmd_part, json_part_with_star = core.split("@", 1)
            json_part = json_part_with_star.split("*", 1)[0]  # 去掉 *
            

            parsed = json.loads(json_part)
            data = parsed.get("data")
            return cmd_part, data

        except Exception as e:
            print(f"❌ decode_message 錯誤：{e}")
            return None, None

    
    def UI_is_ready(self):
        self.is_UI_ready = 1
    
    def stop(self):
        self._running = False
        try:
            self.socket.close()
        except:
            pass
