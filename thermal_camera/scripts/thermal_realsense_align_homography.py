import cv2
import numpy as np
import os

# === 設定 ===
index = 26  # 你要對齊第幾組影像（可改）
folder = "/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046"  # 儲存影像與點位的資料夾名稱

# === 讀取影像與角點 ===
thermal_img = cv2.imread(os.path.join(folder, f"thermal_{index}.jpg"))
realsense_img = cv2.imread(os.path.join(folder, f"realsense_{index}.jpg"))

if thermal_img is None or realsense_img is None:
    print("❌ 無法讀取影像，請確認路徑與檔名")
    exit()

data = np.load(os.path.join(folder, f"points_{index}.npz"))
thermal_pts = data["thermal_img_points"]
realsense_pts = data["realsense_img_points"]

# === 計算 Homography ===
H, mask = cv2.findHomography(thermal_pts, realsense_pts, cv2.RANSAC)

# === 對齊熱像儀影像 ===
aligned_thermal = cv2.warpPerspective(thermal_img, H, (realsense_img.shape[1], realsense_img.shape[0]))

# === 疊圖檢查 ===
overlay = cv2.addWeighted(aligned_thermal, 0.5, realsense_img, 0.5, 0)

# === 顯示結果 ===
cv2.namedWindow("Thermal Aligned", cv2.WINDOW_NORMAL)
cv2.namedWindow("Overlay", cv2.WINDOW_NORMAL)
cv2.imshow("Thermal Aligned", aligned_thermal)
cv2.imshow("Overlay", overlay)

# === 儲存結果 ===
cv2.imwrite(os.path.join(folder, f"thermal_aligned_{index}.jpg"), aligned_thermal)
cv2.imwrite(os.path.join(folder, f"overlay_{index}.jpg"), overlay)

print(f"✅ 對齊完成，結果已儲存：thermal_aligned_{index}.jpg 與 overlay_{index}.jpg")

cv2.waitKey(0)
cv2.destroyAllWindows()
