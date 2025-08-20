# import pyrealsense2 as rs
# import numpy as np
# import cv2

# # 初始化RealSense管道
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# # 開始管道
# profile = pipeline.start(config)

# # 使用 RealSense 的 align 來對齊深度影像和RGB影像
# align_to = rs.stream.color
# align = rs.align(align_to)

# try:
#     while True:
#         # 在循環中捕獲影像
#         frames = pipeline.wait_for_frames()
#         aligned_frames = align.process(frames)  # 將深度影像與RGB影像對齊

#         # 獲取對齊後的RGB與深度影像
#         aligned_depth_frame = aligned_frames.get_depth_frame()
#         aligned_color_frame = aligned_frames.get_color_frame()

#         # 檢查是否成功獲取影像
#         if not aligned_depth_frame or not aligned_color_frame:
#             print("Failed to capture frames")
#             continue

#         # 轉換成numpy數組
#         depth_image = np.asanyarray(aligned_depth_frame.get_data())
#         color_image = np.asanyarray(aligned_color_frame.get_data())

#         # 顯示深度影像（需要將16位深度轉換為8位並應用顏色映射）
#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
#         # 顯示對齊後的RGB與深度影像
#         cv2.imshow("Aligned Depth Image", depth_colormap)
#         cv2.imshow("Aligned Color Image", color_image)
        
#         # 等待鍵盤按鍵，按下 'q' 可退出
#         key = cv2.waitKey(1)
#         if key == ord('q'):
#             break

# finally:
#     # 結束前停止管道
#     pipeline.stop()
#     cv2.destroyAllWindows()
import pyrealsense2 as rs
import numpy as np
import cv2

# 初始化 RealSense 管線與配置
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# 開啟串流
pipeline.start(config)

# 設定對齊目標為 color
align = rs.align(rs.stream.color)

try:
    while True:
        # 原始 frame
        frames = pipeline.wait_for_frames()
        color_frame_raw = frames.get_color_frame()  # 原始 RGB frame

        # 對齊後的 frame
        aligned_frames = align.process(frames)
        color_frame_aligned = aligned_frames.get_color_frame()

        # 驗證
        if not color_frame_raw or not color_frame_aligned:
            continue

        # 轉成 numpy 格式
        color_raw = np.asanyarray(color_frame_raw.get_data())
        color_aligned = np.asanyarray(color_frame_aligned.get_data())

        # === 顯示對齊前後差異（左：原始，右：對齊後） ===
        combined = np.hstack((color_raw, color_aligned))
        cv2.imshow("Raw RGB (left) vs Aligned RGB (right)", combined)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
