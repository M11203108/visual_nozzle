#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    # === 初始化 RealSense 管線 ===
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    # === 啟動串流 ===
    pipeline.start(config)
    depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # === 轉換為 numpy array ===
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # === 深度轉換為可視化圖（未校正） ===
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # === 串接顯示 ===
            combined = np.hstack((color_image, depth_colormap))
            cv2.imshow("Color (Left) + Raw Depth (Right, unaligned)", combined)

            # === 按 ESC 離開 ===
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
