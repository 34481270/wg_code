import os
import math
import requests

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.log(math.tan(lat_rad) + 1 / math.cos(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def download_tiles(lat_min, lon_min, lat_max, lon_max, zoom):
    x_start, y_start = deg2num(lat_max, lon_min, zoom)
    x_end, y_end = deg2num(lat_min, lon_max, zoom)

    print(f"🧭 Tile 範圍 X: {x_start} ~ {x_end}, Y: {y_start} ~ {y_end}, zoom={zoom}")

    headers = {
        'User-Agent': 'Mozilla/5.0 (compatible; MyLeafletDownloader/1.0)'  # 模擬正常瀏覽器
    }

    count = 0
    for x in range(x_start, x_end + 1):
        for y in range(y_start, y_end + 1):
            url = f"https://tile.openstreetmap.org/{zoom}/{x}/{y}.png"
            filename = f"tiles/{zoom}/{x}/{y}.png"
            os.makedirs(os.path.dirname(filename), exist_ok=True)

            print(f"📥 嘗試下載 {url}")
            try:
                r = requests.get(url, headers=headers, timeout=10)
                if r.status_code == 200:
                    with open(filename, "wb") as f:
                        f.write(r.content)
                    print(f"✅ 已儲存：{filename}")
                    count += 1
                else:
                    print(f"⚠️ 失敗（{r.status_code}）：{url}")
            except Exception as e:
                print(f"❌ 錯誤：{e}")
    
    print(f"✅ 共成功儲存 {count} 張圖磚")

# ✅ 測試範圍：基隆外海 zoom 13
download_tiles(
    lat_min=25.01,
    lon_min=121.52,
    lat_max=25.03,
    lon_max=121.542,
    zoom=20
)
# 📥 嘗試下載 https://tile.openstreetmap.org/16/54959/28027.png
# ⚠️ 失敗（503）：https://tile.openstreetmap.org/16/54959/28027.png