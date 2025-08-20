import numpy as np
import cv2

# === 載入 stereo calibration 結果 ===
data = np.load('/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046/stereo_result.npz')
K1, D1 = data['K1'], data['D1']
K2, D2 = data['K2'], data['D2']
R, T  = data['R'], data['T']

# === 構建投影矩陣 P1, P2 ===
P1 = K1 @ np.hstack((np.eye(3), np.zeros((3,1))))        # 熱像儀相機矩陣 [I | 0]
P2 = K2 @ np.hstack((R, T))                              # RealSense 相機矩陣 [R | T]

# === 🔽 這裡替換成你要對應的點 ===
pt_thermal = np.array([320, 240], dtype=np.float32)      # 熱像儀上的像素點（u, v）
pt_realsense = np.array([315, 238], dtype=np.float32)    # RealSense 上對應點（需先對齊過）

# 轉換為齊次座標格式給 triangulatePoints 用
pt_thermal = pt_thermal.reshape(2, 1)
pt_realsense = pt_realsense.reshape(2, 1)

# === 三角測距，取得空間座標 ===
point_4d_hom = cv2.triangulatePoints(P1, P2, pt_thermal, pt_realsense)  # 4x1
point_3d = (point_4d_hom / point_4d_hom[3])[:3].reshape(-1)              # 正規化為 3D

print(f"🎯 該熱點對應 3D 空間座標 (以 RealSense 為基準):\n{point_3d}")
