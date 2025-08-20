#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import os
from visualization_msgs.msg import Marker

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        self.bridge = CvBridge()
        self.marker_pub = self.create_publisher(Marker, 'fire_marker', 10)
        self.depth_colormap_pub = self.create_publisher(Image, 'depth_colormap', 10)

        # === 載入 YOLO 模型 ===
        base_path = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(base_path, 'best.pt')
        self.model = YOLO(model_path)

        # === 初始化 RealSense 串流 ===
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.pipeline.start(self.config)

        # === 使用 align 來對齊深度影像和RGB影像 ===
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # === 深度濾波器 ===
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 3)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.spatial.set_option(rs.option.filter_smooth_delta, 20)
        self.temporal = rs.temporal_filter()
        self.temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
        self.temporal.set_option(rs.option.filter_smooth_delta, 20)
        self.hole_filling = rs.hole_filling_filter()
        self.decimation = rs.decimation_filter()
        self.decimation.set_option(rs.option.filter_magnitude, 2)

        # === 加入 RealSense 原生色彩轉換 ===
        self.colorizer = rs.colorizer()

        # === ROS2 Topic ===
        self.image_pub = self.create_publisher(Image, 'fire_detection_image', 10)
        self.depth_pub = self.create_publisher(Float32, 'fire_depth', 10)
        self.pos_pub = self.create_publisher(Point, 'fire_position', 10)

        self.depth_scale = self.pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('🔥 Fire Detection Node 已啟動')

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame_raw = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame_raw:
            return

        depth_frame = self.decimation.process(depth_frame_raw)
        depth_frame = self.spatial.process(depth_frame)
        depth_frame = self.temporal.process(depth_frame)
        depth_frame = self.hole_filling.process(depth_frame)

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # === 用 colorizer 轉換 depth image 為色彩圖 ===
        colorized_depth = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
        depth_colormap_msg = self.bridge.cv2_to_imgmsg(colorized_depth, encoding='bgr8')
        depth_colormap_msg.header.stamp = self.get_clock().now().to_msg()
        depth_colormap_msg.header.frame_id = "camera_link"
        self.depth_colormap_pub.publish(depth_colormap_msg)

        # === YOLO 預測 ===
        results = self.model.predict(source=color_image, show=False, conf=0.4)
        annotated_frame = results[0].plot()

        best_box = None
        max_area = 0
        for box in results[0].boxes:
            if int(box.cls[0]) != 0:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)
            if area > max_area:
                max_area = area
                best_box = (x1, y1, x2, y2)

        if best_box is None:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))
            return

        x1, y1, x2, y2 = best_box
        center_x = (x1 + x2) // 2
        bottom_y = min(y2 + 5, depth_image.shape[0] - 1)
        scale_x = depth_image.shape[1] / color_image.shape[1]
        scaled_center_x = int(center_x * scale_x)

        min_depth = float('inf')
        best_y = bottom_y
        for new_y in range(bottom_y, min(bottom_y + 30, depth_image.shape[0]), 2):
            d = depth_image[new_y, scaled_center_x] * self.depth_scale
            if 0 < d < min_depth:
                min_depth = d
                best_y = new_y

        if min_depth == float('inf'):
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))
            return

        compensated_depth = min_depth + 0.4
        cv2.putText(annotated_frame, f"Depth: {compensated_depth:.2f}m", (x1, y1 - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        self.depth_pub.publish(Float32(data=compensated_depth))
        intrin = color_frame.profile.as_video_stream_profile().intrinsics
        X, Y, Z = rs.rs2_deproject_pixel_to_point(intrin, [center_x, best_y], min_depth)
        X_cam = Z
        Y_cam = -X
        Z_cam = -Y

        self.pos_pub.publish(Point(x=X_cam, y=Y_cam, z=Z_cam))

        marker = Marker()
        marker.header.frame_id = 'camera_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fire'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = X_cam
        marker.pose.position.y = Y_cam
        marker.pose.position.z = Z_cam
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

        self.get_logger().info(f'🔥 Fire Position: ({X_cam:.2f}, {Y_cam:.2f}, {Z_cam:.2f}) m')

        # === 在深度圖中顯示 fire 位置 ===
        depth_annotated_frame = colorized_depth.copy()
        x1_depth = int(x1 * (depth_image.shape[1] / color_image.shape[1]))
        y1_depth = int(y1 * (depth_image.shape[0] / color_image.shape[0]))
        x2_depth = int(x2 * (depth_image.shape[1] / color_image.shape[1]))
        y2_depth = int(y2 * (depth_image.shape[0] / color_image.shape[0]))
        cv2.rectangle(depth_annotated_frame, (x1_depth, y1_depth), (x2_depth, y2_depth), (0, 255, 0), 2)
        cv2.putText(depth_annotated_frame, f"Fire", (x1_depth, y1_depth - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))
        self.depth_colormap_pub.publish(self.bridge.cv2_to_imgmsg(depth_annotated_frame, encoding='bgr8'))

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FireDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
