import numpy as np
import cv2

# 載入 Homography 矩陣
H = np.load('homography_matrix.npy')

# 假設這是你的熱像儀影像
ir_image = cv2.imread('your_thermal_image.png')

# 將影像轉換 (假設 RealSense 是 1280x720)
warped_image = cv2.warpPerspective(ir_image, H, (1280, 720))

cv2.imshow("Warped Thermal Image", warped_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
