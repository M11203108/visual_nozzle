import numpy as np
import cv2

# 載入對應點資料
rs_points = np.load('rs_points.npy')
ir_points = np.load('ir_points.npy')

# 確認對應點的數量一致
assert len(rs_points) == len(ir_points)

# 轉換成 numpy 格式
rs_points = np.array(rs_points, dtype=np.float32)
ir_points = np.array(ir_points, dtype=np.float32)

# 計算 Homography 矩陣
H, status = cv2.findHomography(ir_points, rs_points, cv2.RANSAC, 5.0)

# 儲存 Homography 矩陣
np.save('homography_matrix.npy', H)
print("Homography matrix:\n", H)
