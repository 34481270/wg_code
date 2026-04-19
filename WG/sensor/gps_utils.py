from pyproj import Transformer
from geopy.distance import geodesic

# 台灣用的 EPSG 投影系統：WGS84 經緯度 ↔ UTM zone 51
_TO_UTM = Transformer.from_crs("EPSG:4326", "EPSG:32651", always_xy=True)
_TO_GPS = Transformer.from_crs("EPSG:32651", "EPSG:4326", always_xy=True)


def gps_to_utm(lat: float, lon: float):
    """
    將 (lon, lat) 轉為 UTM (x, y)，單位為公尺。
    """
    x, y = _TO_UTM.transform(lon, lat)
    return x, y

def utm_to_gps(x: float, y: float):
    """
    將 (x, y) UTM 座標轉為 (lon, lat) GPS 經緯度。
    """
    lon, lat = _TO_GPS.transform(x, y)
    return lat, lon

def gps_to_relative_xy(lat0, lon0, lat1, lon1):
    x0, y0 = gps_to_utm(lat0, lon0)  # ✅ 正確順序
    x1, y1 = gps_to_utm(lat1, lon1)

    return x1 - x0, y1 - y0


from geopy.distance import geodesic

def find_nearest_point(lat, lon, path):
    """
    找出距離 (lat, lon) 最近的點在 path 中的位置

    Args:
        lat (float): 當前 GPS 緯度
        lon (float): 當前 GPS 經度
        path (list of [lat, lon]): 路徑點

    Returns:
        dict: {
            "index": 最近點的索引,
            "point": 最近點座標 (lat, lon),
            "distance_m": 距離最近點的距離（公尺）
        }
    """
    min_dist = float('inf')
    nearest_point = None
    nearest_index = -1

    for i, (plat, plon) in enumerate(path):
        d = geodesic((lat, lon), (plat, plon)).meters
        if d < min_dist:
            min_dist = d
            nearest_point = (plat, plon)
            nearest_index = i

    return {
        "index": nearest_index,
        "point": nearest_point,
        "distance_m": min_dist
    }




