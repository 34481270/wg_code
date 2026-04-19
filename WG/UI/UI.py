import sys
project_root = r"C:\Users\david ho\Desktop\WG"
if project_root not in sys.path:
    sys.path.insert(0, project_root)
import os 
import webview
import threading
import json
import socket
from queue import Queue, Empty
from geopy.distance import distance
import numpy as np
from WG.Model import findVirtualTarget  # type: ignore
from pyproj import Proj, Transformer


# 建立 HTML 地圖模板
dir_path = os.path.dirname(__file__)
template_path = os.path.join(dir_path, "template.html")

with open(template_path, "r", encoding="utf-8") as f:
    html_template = f.read()

# with open("dynamic_map.html", "w", encoding="utf-8") as f:
#     f.write(html_template)

print(f"✅ 動態地圖已建立：{os.path.abspath('dynamic_map.html')}\n等待 glider 資料傳入...")

# === API 提供給 JS 呼叫的函式 ===
class API:
    def __init__(self):
        self.socket_conn = None
        self.queue = Queue()  # ✅ 新增 queue 儲存所有資料
        self.pid_window = None
        
        self.js_buffer = []
        self.ui_ready = False    

    def check_socket(self):
        try:
            if self.socket_conn:
                self.socket_conn.sendall(b"ping\n")
                return True
            else:
                return False
        except Exception as e:
            return False
    
    def send_cmd(self, cmd):
        print('送出命令', cmd)
        try:
            if self.socket_conn:
                print('送出命令', cmd)
                self.socket_conn.sendall(cmd.encode())
                return True
            else:
                return False
        except Exception as e:
            return False

    def open_pid(self):
        pid_path = os.path.join(dir_path, "pid.html")
        pid_window = webview.create_window(
            'PID 參數調整',
            pid_path,
            js_api=self,
            width=800,
            height=1600
        )
        self.pid_window = pid_window


# === Socket Server：接收資料、存進 queue ===
# ipconfig getifaddr en0 找Mac的IP
def start_socket_server(window):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", 5005))
    server.listen(1)
    print("🌐 Socket Server listening on port 5005")

    try:
        while True:
            print("🕓 等待 client 連接...")
            try:
                conn, addr = server.accept()
                print(f"✅ Connected by {addr}")
                api.socket_conn = conn

                recv_buffer = ""

                while True:
                    raw_data = conn.recv(16384)
                    if not raw_data:
                        print("⚠️ Client 中斷連線")
                        break

                    recv_buffer += raw_data.decode()

                    # ✅ 每筆資料結尾一定是 '\n'，所以可以分段處理
                    while "\n" in recv_buffer:
                        line, recv_buffer = recv_buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            cmd, data = decode_message(line)
                            if cmd is not None:
                                print(f" 收到指令：{cmd}")
                                api.queue.put((cmd, data))
                            else:
                                print(f"⚠️ 無法解析指令：{line}")
                        except Exception as e:
                            print(f"❌ 處理錯誤：{e}")
            except Exception as e:
                print(f"❌ 接收 client 發生錯誤：{e}")
            finally:
                try:
                    conn.close()
                except:
                    pass
                api.socket_conn = None
                print("🔌 已關閉 client，重新等待下一位")
    finally:
        print("🧹 結束時關閉 server socket")
        server.close()


# === 背景 Thread：從 queue 取資料並處理 ===
def process_queue_data(window):
    def drawCurrentPathTrajectory(data):
        points = data
        js = f"drawCurrentPathTrajectory({json.dumps(points)}, '{'blue'}')"
        window.evaluate_js(js)
        
    def drawDubinsPathTrajectory(data):
        points = data
        js = f"drawDubinsPathTrajectory({json.dumps(points)})"
        window.evaluate_js(js)

    def add_point(data):
        lat = data[0]
        lon = data[1]
        if lat is not None and lon is not None:
            js = f"addPoint({lat}, {lon})"
            window.evaluate_js(js)
    
    def noGoal(data):
        js = "noGoal()"
        window.evaluate_js(js)
        
    def drawSimulate(data):
        points = data
        js = f"drawSimulate({json.dumps(points)}, '{'blue'}')"
        window.evaluate_js(js)


    def State(data):
        # 假設 data 本身就是要顯示的文字，例如 "Navigating"
        js_arg = json.dumps(data)          # → '"Navigating"', 自動加上雙引號並處理跳脫
        js     = f"addState({js_arg})"      # → addState("Navigating")
        window.evaluate_js(js)

    def updateStats(data):
        js = f"updateStats({json.dumps(data)})"
        window.evaluate_js(js)
        
    def indicator(data):
        propeller = data.get('propeller', 0)
        navigation = data.get('navigation', 0)
        js = f"propeller({json.dumps(propeller)})"
        window.evaluate_js(js)
        js = f"navigation({json.dumps(navigation)})"
        window.evaluate_js(js)
        
    def pid_data(data):
        js = f"pid_data({json.dumps(data)})"
        api.pid_window.evaluate_js(js)
        
    def course_angle(data):
        try:
            js = f"course_angle({json.dumps(data)})"
            api.pid_window.evaluate_js(js)
        except:
            print("尚未開啟PID測試頁面")
            
    def drawArrow(data):
        js = f"drawArrow({json.dumps(data)})"
        window.evaluate_js(js)
        
    def drawOrigin(data):
        js = f"drawOrigin({json.dumps(data)})"
        window.evaluate_js(js)

    handler_map = {
        "drawCurrentPathTrajectory": drawCurrentPathTrajectory,
        "drawDubinsPathTrajectory": drawDubinsPathTrajectory,
        'drawSimulate' : drawSimulate,
        "addPoint": add_point,
        "noGoal" : noGoal,
        'State' : State,
        'updateStats': updateStats,
        'indicator': indicator,
        'pid_data':pid_data,
        'course_angle' : course_angle,
        'drawArrow' : drawArrow,
        'drawOrigin' : drawOrigin,
    }

    while True:
        try:
            item = api.queue.get_nowait()

            if isinstance(item, tuple) and len(item) == 2:
                cmd, data = item
                handler = handler_map.get(cmd)
                if handler:
                    handler(data)
                else:
                    print(f"⚠️ 未知指令：{cmd}")
        except Empty:
            continue



def decode_message(message):
    """
    從自訂協議格式的訊息中解析出 cmd 和 data。
    預期格式：$<cmd>@<json_data>*\\n

    Returns:
        tuple: (cmd, data)
    """
    try:
        if not message.startswith("$") or "@" not in message or "*" not in message:
            raise ValueError("格式錯誤")

        # 先去掉開頭 $，然後切開 cmd 和 json
        core = message.strip()[1:]  # 移除開頭 $ 和結尾換行
        cmd_part, json_part_with_star = core.split("@", 1)
        json_part = json_part_with_star.split("*", 1)[0]  # 移除 *

        parsed = json.loads(json_part)
        data = parsed.get("data")

        return cmd_part, data

    except Exception as e:
        print(f"❌ decode_message 錯誤：{e}")
        return None, None



# === 啟動主程式 ===
if __name__ == '__main__':
    api = API()
    window = webview.create_window("Dynamic Glider Map", template_path, js_api=api,width=1200,height=800)

    threading.Thread(target=start_socket_server, args=(window,), daemon=True).start()
    threading.Thread(target=process_queue_data, args=(window,), daemon=True).start()

    webview.start()