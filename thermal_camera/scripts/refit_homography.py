import os
import numpy as np
import cv2

# === 參數 ===
FOLDER = "/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046"
THRESH = 20.0  # 最大允許平均誤差

# === 初始化 ===
good_thermal = []
good_realsense = []

print("📊 開始篩選 Homography 誤差...")

# 讀取每組角點
for fname in sorted(os.listdir(FOLDER)):
    if fname.startswith("points_") and fname.endswith(".npz"):
        idx = fname.split("_")[1].split(".")[0]
        data = np.load(os.path.join(FOLDER, fname), allow_pickle=True)
        thermal_pts = data["thermal_img_points"].reshape(-1, 2)
        realsense_pts = data["realsense_img_points"].reshape(-1, 2)

        # 嘗試擬合此組對應的 Homography
        H_temp, _ = cv2.findHomography(thermal_pts, realsense_pts, cv2.RANSAC)

        # 投影並計算誤差
        thermal_proj = cv2.perspectiveTransform(thermal_pts.reshape(-1, 1, 2), H_temp).reshape(-1, 2)
        errors = np.linalg.norm(realsense_pts - thermal_proj, axis=1)
        mean_err = np.mean(errors)

        if mean_err < THRESH:
            good_thermal.append(thermal_pts)
            good_realsense.append(realsense_pts)
            print(f"✅ 圖 {idx:>2} 保留 (avg error={mean_err:.2f} px)")
        else:
            print(f"❌ 圖 {idx:>2} 捨棄 (avg error={mean_err:.2f} px)")

# === 整合所有保留的點並重新計算 H ===
if len(good_thermal) < 5:
    print("⚠️ 可用樣本太少，無法重新估算 Homography")
else:
    all_thermal = np.vstack(good_thermal)
    all_realsense = np.vstack(good_realsense)
    H_new, _ = cv2.findHomography(all_thermal, all_realsense, cv2.RANSAC)
    np.save(os.path.join(FOLDER, "H_avg.npy"), H_new)
    print(f"\n💾 已更新 Homography → {FOLDER}/H_avg.npy")
    print("🎯 重新擬合完成")
