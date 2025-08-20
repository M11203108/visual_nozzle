import os
import glob
import numpy as np
import cv2

# === 參數設定 ===
npz_folder = "/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046"
threshold_px = 1.5  # 🔧 篩選條件：平均誤差小於此值的資料會被納入平均 H 計算

# === 開始載入與分析 ===
npz_files = sorted(glob.glob(os.path.join(npz_folder, "points_*.npz")))
all_errors = []
thermal_pts_all = []
realsense_pts_all = []

print(f"📂 載入 {len(npz_files)} 組點位進行 Homography 誤差評估與篩選")

for npz_file in npz_files:
    data = np.load(npz_file)
    thermal_pts = data['thermal_img_points'].reshape(-1, 2)
    realsense_pts = data['realsense_img_points'].reshape(-1, 2)

    H, _ = cv2.findHomography(thermal_pts, realsense_pts, cv2.RANSAC)

    ones = np.ones((thermal_pts.shape[0], 1))
    thermal_homogeneous = np.hstack([thermal_pts, ones])
    projected = (H @ thermal_homogeneous.T).T
    projected /= projected[:, 2][:, np.newaxis]
    predicted_realsense = projected[:, :2]

    errors = np.linalg.norm(predicted_realsense - realsense_pts, axis=1)
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    print(f"{os.path.basename(npz_file)}: 平均誤差 = {mean_error:.2f} px, 最大誤差 = {max_error:.2f} px")

    if mean_error < threshold_px:
        thermal_pts_all.append(thermal_pts)
        realsense_pts_all.append(realsense_pts)

    all_errors.extend(errors)

# === 計算與儲存平均 Homography ===
if thermal_pts_all:
    thermal_pts_all = np.vstack(thermal_pts_all)
    realsense_pts_all = np.vstack(realsense_pts_all)
    H_avg, _ = cv2.findHomography(thermal_pts_all, realsense_pts_all, cv2.RANSAC)
    np.save("H_avg.npy", H_avg)
    print(f"\n✅ 已儲存平均 Homography 為 H_avg.npy，共使用 {len(thermal_pts_all)} 個點")
else:
    print("\n⚠️ 沒有符合誤差門檻的點組，無法建立平均 Homography。")

# === 顯示總體統計 ===
all_errors = np.array(all_errors)
print("\n📊 總體 Homography 誤差統計：")
print(f"➡️  平均誤差：{np.mean(all_errors):.2f} px")
print(f"➡️  最大誤差：{np.max(all_errors):.2f} px")
print(f"➡️  最小誤差：{np.min(all_errors):.2f} px")
print(f"➡️  標準差：{np.std(all_errors):.2f} px")
