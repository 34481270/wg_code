import numpy as np
from geopy.distance import geodesic
import numpy as np
import math

def findHeading(current_path, position, dubins_path):

    min_dist = float('inf')
    idx = -1

    for i, (plat, plon) in enumerate(current_path):
        d = geodesic((position[0], position[1]), (plat, plon)).meters
        if d < min_dist:
            min_dist = d
            idx = i

    if idx == len(dubins_path) - 1:
        heading = np.arctan2(
            current_path[-1, 1] - position[1],
            current_path[-1, 0] - position[0]
        )
    else:
        heading = np.arctan2(
            dubins_path[idx + 1, 1] - dubins_path[idx, 1],
            dubins_path[idx + 1, 0] - dubins_path[idx, 0]
        )

    heading_deg = (math.degrees(heading) + 360) % 360

    return min_dist, heading_deg