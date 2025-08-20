import numpy as np
import cv2

# === 載入 stereo calibration 結果 ===
calib_file = "/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046/stereo_result.npz"
data = np.load(calib_file)
K1, D1 = data['K1'], data['D1']   # Thermal
K2, D2 = data['K2'], data['D2']   # RealSense
R, T = data['R'], data['T']

# === 計算投影矩陣 ===
P1 = K1 @ np.hstack((np.eye(3), np.zeros((3, 1))))   # 熱像儀
P2 = K2 @ np.hstack((R, T))                          # RealSense

def triangulate_thermal_to_3d(thermal_px, realsense_px):
    """
    thermal_px:     (x, y) 像素座標 from thermal image
    realsense_px:   (x, y) 像素座標 from RGB image (after homography or feature matching)

    return: (x, y, z) 空間座標（以熱像儀 frame 為基準）
    """
    pt1 = np.array(thermal_px, dtype=np.float32).reshape(2, 1)
    pt2 = np.array(realsense_px, dtype=np.float32).reshape(2, 1)

    # Triangulate
    point_4d = cv2.triangulatePoints(P1, P2, pt1, pt2)
    point_3d = point_4d[:3] / point_4d[3]

    return point_3d.flatten()

# === 🔥 測試用：一對對應點 ===
thermal_point = (240.0, 120.0)     # 最熱點 in 熱像儀
realsense_point = (325.0, 140.0)   # 映射對應的 RealSense 像素點

point_3d = triangulate_thermal_to_3d(thermal_point, realsense_point)
print(f"🧮 計算得到的 3D 座標 (thermal frame): {point_3d}")
