import os
import numpy as np
import cv2

# === 你存檔的資料夾路徑 ===
# npz_folder = 'dual_calib_20250628_123046'  # ← 根據實際資料夾修改
npz_folder = '/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046'


# === 初始化 ===
objpoints = []
imgpoints_thermal = []
imgpoints_realsense = []

# === 讀取所有 npz 檔 ===
for filename in sorted(os.listdir(npz_folder)):
    if filename.startswith("points_") and filename.endswith(".npz"):
        data = np.load(os.path.join(npz_folder, filename))
        objpoints.append(data['obj_points'])
        imgpoints_thermal.append(data['thermal_img_points'])
        imgpoints_realsense.append(data['realsense_img_points'])

print(f"✅ 載入 {len(objpoints)} 組資料進行 stereo calibration")

# === 從單相機標定內參開始 ===
image_shape = (640, 480)  # 或改為你原始圖像大小

# 預設 focal length fx, fy, cx, cy 的起始值
K1 = np.array([[500, 0, image_shape[0]/2],
               [0, 500, image_shape[1]/2],
               [0, 0, 1]], dtype=np.float64)
D1 = np.zeros((5, 1))  # 熱像儀的畸變參數

K2 = np.array([[500, 0, image_shape[0]/2],
               [0, 500, image_shape[1]/2],
               [0, 0, 1]], dtype=np.float64)
D2 = np.zeros((5, 1))  # RealSense 的畸變參數

# === 執行 stereoCalibrate ===
flags = cv2.CALIB_FIX_INTRINSIC  # 假設你已經個別標定過兩個相機，可以改為 0 讓系統自動估計
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 100, 1e-6)

ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpoints_thermal, imgpoints_realsense,
    K1, D1, K2, D2, image_shape,
    criteria=criteria,
    flags=flags
)

# === 顯示結果 ===
print(f"\n🎯 Stereo Calibration Completed")
print(f"R (旋轉矩陣):\n{R}")
print(f"T (平移向量):\n{T}")
print(f"重投影誤差 (reprojection error): {ret}")

# === 儲存結果 ===
np.savez(os.path.join(npz_folder, 'stereo_calibration_result.npz'),
         K1=K1, D1=D1,
         K2=K2, D2=D2,
         R=R, T=T,
         E=E, F=F,
         error=ret)

print(f"\n✅ 結果已儲存到 {npz_folder}/stereo_calibration_result.npz")
