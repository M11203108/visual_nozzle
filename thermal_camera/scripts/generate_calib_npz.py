import numpy as np

# 這裡請填入你個別相機標定後的內參
K1 = np.array([[502.3, 0, 319.5],
               [0, 503.1, 239.7],
               [0, 0, 1]])
D1 = np.zeros((5, 1))  # 若無畸變請填 0

K2 = np.array([[610.2, 0, 320.1],
               [0, 610.5, 240.4],
               [0, 0, 1]])
D2 = np.zeros((5, 1))  # 若無畸變請填 0

# 圖像尺寸 (width, height)
image_size = (640, 480)

# 儲存為 npz
np.savez('/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046/calibration_data.npz',
         K1=K1, D1=D1,
         K2=K2, D2=D2,
         image_size=image_size)

print("✅ calibration_data.npz 已成功建立")
