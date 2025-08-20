import os
import glob
import numpy as np
import cv2

# 修改這裡指定資料夾路徑（含 points_*.npz）
npz_folder = "/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046"  # 可自行修改
npz_files = sorted(glob.glob(os.path.join(npz_folder, "points_*.npz")))

all_errors = []

print(f"📂 載入 {len(npz_files)} 組點位進行 Homography 誤差評估")

for npz_file in npz_files:
    data = np.load(npz_file)
    thermal_pts = data['thermal_img_points'].reshape(-1, 2)
    realsense_pts = data['realsense_img_points'].reshape(-1, 2)


    # 計算 Homography：thermal → realsense
    H, mask = cv2.findHomography(thermal_pts, realsense_pts, cv2.RANSAC)

    # 使用 H 將 thermal_pts 轉換為 realsense 座標系
    ones = np.ones((thermal_pts.shape[0], 1))
    thermal_homogeneous = np.hstack([thermal_pts, ones])  # shape (N, 3)
    projected = (H @ thermal_homogeneous.T).T  # shape (N, 3)
    projected /= projected[:, 2][:, np.newaxis]  # 正規化

    predicted_realsense = projected[:, :2]  # shape (N, 2)

    # 計算像素誤差（每個點）
    errors = np.linalg.norm(predicted_realsense - realsense_pts, axis=1)

    # 顯示每組誤差統計
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    print(f"{os.path.basename(npz_file)}: 平均誤差 = {mean_error:.2f} px, 最大誤差 = {max_error:.2f} px")

    all_errors.extend(errors)

# 統計全部誤差
all_errors = np.array(all_errors)
print("\n📊 總體 Homography 誤差統計：")
print(f"➡️  平均誤差：{np.mean(all_errors):.2f} px")
print(f"➡️  最大誤差：{np.max(all_errors):.2f} px")
print(f"➡️  最小誤差：{np.min(all_errors):.2f} px")
print(f"➡️  標準差：{np.std(all_errors):.2f} px")
