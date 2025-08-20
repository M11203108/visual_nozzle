import os
import cv2
import numpy as np

folder = "/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046"
calib = np.load(os.path.join(folder, "stereo_result.npz"))
H = calib["H"]

images = []
titles = []

for fname in sorted(os.listdir(folder)):
    if fname.startswith("points_") and fname.endswith(".npz"):
        idx = fname.split("_")[1].split(".")[0]

        data = np.load(os.path.join(folder, fname), allow_pickle=True)
        thermal_pts = data["thermal_img_points"].reshape(-1, 2)
        realsense_pts = data["realsense_img_points"].reshape(-1, 2)

        thermal_img_path = os.path.join(folder, f"thermal_{idx}.jpg")
        realsense_img_path = os.path.join(folder, f"realsense_{idx}.jpg")

        thermal_img = cv2.imread(thermal_img_path)
        realsense_img = cv2.imread(realsense_img_path)

        if thermal_img is None or realsense_img is None:
            print(f"❌ 找不到影像 thermal_{idx}.jpg 或 realsense_{idx}.jpg")
            continue

        thermal_pts_homo = cv2.perspectiveTransform(thermal_pts.reshape(-1, 1, 2), H).reshape(-1, 2)

        for p in realsense_pts:
            p = tuple(np.int32(p))
            cv2.circle(realsense_img, p, 5, (0, 255, 0), -1)

        for p in thermal_pts_homo:
            p = tuple(np.int32(p))
            cv2.circle(realsense_img, p, 5, (0, 0, 255), 2)

        for p1, p2 in zip(realsense_pts, thermal_pts_homo):
            err = np.linalg.norm(p1 - p2)
            cv2.line(realsense_img, tuple(np.int32(p1)), tuple(np.int32(p2)), (255, 0, 0), 1)
            cv2.putText(realsense_img, f"{err:.1f}", tuple(np.int32(p2 + np.array([5, -5]))),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        images.append(realsense_img)
        titles.append(f"Index: {idx}")

# === 顯示並切換所有畫面 ===
i = 0
while True:
    img = images[i].copy()
    cv2.putText(img, f"[{i+1}/{len(images)}] - {titles[i]}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imshow("Stereo Point Check", img)
    key = cv2.waitKey(0)

    if key == ord('q') or key == 27:  # q or ESC
        break
    elif key == 81 or key == ord('a'):  # ←
        i = (i - 1) % len(images)
    elif key == 83 or key == ord('d'):  # →
        i = (i + 1) % len(images)

cv2.destroyAllWindows()
# import os
# import cv2
# import numpy as np
# from collections import defaultdict

# folder = "/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046"
# calib = np.load(os.path.join(folder, "stereo_result.npz"))
# H = calib["H"]

# heat_map = defaultdict(list)  # (x,y) → list of errors

# for fname in sorted(os.listdir(folder)):
#     if fname.startswith("points_") and fname.endswith(".npz"):
#         idx = fname.split("_")[1].split(".")[0]
#         data = np.load(os.path.join(folder, fname), allow_pickle=True)
#         thermal_pts = data["thermal_img_points"].reshape(-1, 2)
#         realsense_pts = data["realsense_img_points"].reshape(-1, 2)

#         thermal_pts_homo = cv2.perspectiveTransform(thermal_pts.reshape(-1, 1, 2), H).reshape(-1, 2)

#         for (t_pt, r_pt, h_pt) in zip(thermal_pts, realsense_pts, thermal_pts_homo):
#             err = np.linalg.norm(h_pt - r_pt)
#             x, y = int(round(t_pt[0])), int(round(t_pt[1]))
#             heat_map[(x, y)].append(err)

# # === 統整成誤差圖 ===
# width, height = 160, 120  # 熱像儀解析度
# error_map = np.zeros((height, width), dtype=np.float32)
# count_map = np.zeros((height, width), dtype=np.int32)

# for (x, y), errs in heat_map.items():
#     if 0 <= x < width and 0 <= y < height:
#         error_map[y, x] = np.mean(errs)
#         count_map[y, x] = len(errs)

# # 填補沒有誤差資料的區域（可選，這裡用最近的非零平均）
# mask = (count_map == 0)
# if np.any(mask):
#     from scipy.ndimage import distance_transform_edt

#     dist, indices = distance_transform_edt(mask, return_indices=True)
#     error_map[mask] = error_map[tuple(indices[:, mask])]

# # 儲存誤差圖
# np.save(os.path.join(folder, "H_error_map.npy"), error_map)
# print("✅ H_error_map.npy 已儲存")
