import numpy as np
import math

# 基本関数
# 正規化
def min_max_normalize(data):

    data = np.array(data)

    max_data = max(data)
    min_data = min(data)

    if max_data - min_data == 0:
        data = [0.0 for i in range(len(data))]
    else:
        data = (data - min_data) / (max_data - min_data)

    return data
# 角度補正用
def angle_range_corrector(angle):

    if angle > math.pi:
        while angle > math.pi:
            angle -=  2 * math.pi
    elif angle < -math.pi:
        while angle < -math.pi:
            angle += 2 * math.pi

    return angle

