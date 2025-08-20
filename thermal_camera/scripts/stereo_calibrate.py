import os
import numpy as np
import cv2

# === [1] 設定資料夾 ===
folder = "/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046"

# === [2] 匯入所有角點 ===
objpoints = []
thermal_points = []
realsense_points = []

for fname in sorted(os.listdir(folder)):
    if fname.startswith("points_") and fname.endswith(".npz"):
        data = np.load(os.path.join(folder, fname), allow_pickle=True)
        objpoints.append(data["obj_points"])
        thermal_points.append(data["thermal_img_points"])
        realsense_points.append(data["realsense_img_points"])

print(f"✅ 匯入 {len(objpoints)} 組角點")

# === [3] 讀取單機標定的內參與 image size ===
calib = np.load(os.path.join(folder, "calibration_data.npz"))
K1 = calib["K1"]
D1 = calib["D1"]
K2 = calib["K2"]
D2 = calib["D2"]
image_size = tuple(calib["image_size"])  # e.g., (640, 480)

# === [4] 執行 stereo calibration，固定內參，只估外參 ===
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
flags = cv2.CALIB_FIX_INTRINSIC

ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
    objpoints, thermal_points, realsense_points,
    K1, D1, K2, D2, image_size,
    criteria=criteria, flags=flags
)

# === [5] 顯示 stereo calibration 結果 ===
print(f"\n🎯 Stereo Calibration Completed")
print(f"📐 重投影誤差 (RMS error): {ret:.4f}")
print(f"🌀 Rotation Matrix R:\n{R}")
print(f"➡️ Translation Vector T:\n{T}")

# === [6] 產生 Homography: 熱像儀 → RealSense 映射矩陣 H ===
all_thermal = np.vstack(thermal_points)
all_realsense = np.vstack(realsense_points)
H, mask = cv2.findHomography(all_thermal, all_realsense)

print(f"\n🧭 Homography matrix H (thermal → realsense):\n{H}")

# === [7] 儲存所有結果 ===
save_path = os.path.join(folder, "stereo_result.npz")
np.savez(save_path,
         K1=K1, D1=D1,
         K2=K2, D2=D2,
         R=R, T=T,
         E=E, F=F,
         H=H,
         error=ret)

print(f"\n✅ 已儲存到 {save_path}")
