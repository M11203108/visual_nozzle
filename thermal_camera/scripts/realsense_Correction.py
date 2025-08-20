import pyrealsense2 as rs
import numpy as np
import cv2

# === 1. 建立 RealSense 管線 ===
pipeline = rs.pipeline()
config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)


# === 2. 濾波器初始化 ===
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 3)
spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
spatial.set_option(rs.option.filter_smooth_delta, 50)

temporal = rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha, 0.6)
temporal.set_option(rs.option.filter_smooth_delta, 10)

hole_filling = rs.hole_filling_filter()

decimation = rs.decimation_filter()
decimation.set_option(rs.option.filter_magnitude, 2)

# === 3. 開始串流 ===
pipeline.start(config)

# === 4. 取得深度比例（用於轉換深度值為公尺）===
depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()

# === 5. 滑鼠點擊取得深度 ===
clicked_point = None

def mouse_callback(event, x, y, flags, param):
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)

cv2.namedWindow('Filtered Depth')
cv2.setMouseCallback('Filtered Depth', mouse_callback)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        # === 6. 濾波處理 ===
        filtered = decimation.process(depth_frame)
        filtered = spatial.process(filtered)
        filtered = temporal.process(filtered)
        filtered = hole_filling.process(filtered)

        # === 7. 轉為彩色深度圖像 ===
        colorizer = rs.colorizer()
        depth_color_frame = colorizer.colorize(filtered)
        depth_color_image = np.asanyarray(depth_color_frame.get_data())

        # === 8. 顯示點選的深度值 ===
        if clicked_point is not None:
            x, y = clicked_point
            depth_image = np.asanyarray(filtered.get_data())
            if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
                raw_depth = depth_image[y, x]
                distance_m = raw_depth * depth_scale
                cv2.circle(depth_color_image, (x, y), 5, (0, 255, 0), -1)
                cv2.putText(depth_color_image, f"{distance_m:.3f} m", (x + 10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


        # === 9. 顯示結果畫面 ===
        cv2.imshow('Filtered Depth', depth_color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
