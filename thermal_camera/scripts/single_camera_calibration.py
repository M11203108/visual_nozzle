import numpy as np

# 讀取內部校正結果
calib_file = '/home/robot/newjasmine_ws/src/thermal_camera/scripts/calibration_20250621_152657/calibration_data.npz'  # 替換成你實際資料夾
data = np.load(calib_file)

camera_matrix = data['camera_matrix']
dist_coeffs = data['dist_coeffs']

print("相機內參 (camera_matrix):\n", camera_matrix)
print("畸變係數 (dist_coeffs):\n", dist_coeffs)


# import numpy as np
# import cv2
# import os

# folder = '/home/robot/newjasmine_ws/src/thermal_camera/scripts/calibration_20250621_152657'
# image_files = sorted([f for f in os.listdir(folder) if f.endswith('.jpg')])
# img = cv2.imread(os.path.join(folder, image_files[0]))
# image_size = img.shape[1], img.shape[0]  # (width, height)

# data = np.load(os.path.join(folder, 'calibration_data.npz'))
# np.savez(os.path.join(folder, 'calibration_data.npz'),
#          camera_matrix=data['camera_matrix'],
#          dist_coeffs=data['dist_coeffs'],
#          image_size=image_size)

# print("✅ 已補上 image_size =", image_size)
