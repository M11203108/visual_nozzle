#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import numpy as np
import cv2
import pyrealsense2 as rs
import os
from ultralytics import YOLO

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        self.bridge = CvBridge()
        self.model = YOLO(os.path.join(os.path.dirname(__file__), 'best.pt'))

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.pipeline.start(config)

        self.align = rs.align(rs.stream.color)
        self.decimation = rs.decimation_filter()
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        self.hole_filling = rs.hole_filling_filter()
        self.colorizer = rs.colorizer()
        self.depth_scale = self.pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()

        # ROS2 Publishers
        self.image_pub = self.create_publisher(Image, 'fire_detection_image', 10)
        self.depth_colormap_pub = self.create_publisher(Image, 'depth_colormap', 10)
        self.depth_pub = self.create_publisher(Float32, 'fire_depth', 10)
        self.pos_pub = self.create_publisher(Point, 'fire_position', 10)
        self.marker_pub = self.create_publisher(Marker, 'fire_marker', 10)

        # Thermal info subscriber
        self.thermal_pixel_sub = self.create_subscription(
            Float32MultiArray, '/thermal_to_rgb_pixel', self.thermal_pixel_callback, 10)
        self.thermal_rgb_pixel = None
        self.thermal_bbox = None

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('🔥 Fire Detection Node 啟動')

    def thermal_pixel_callback(self, msg):
        if len(msg.data) >= 6:
            self.thermal_rgb_pixel = (int(msg.data[0]), int(msg.data[1]))
            self.thermal_bbox = tuple(map(int, msg.data[2:6]))
            self.get_logger().info(f"📌 Thermal 框與點接收: pt=({self.thermal_rgb_pixel}), box={self.thermal_bbox}")

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        # 深度濾波
        depth_frame = self.decimation.process(depth_frame)
        depth_frame = self.spatial.process(depth_frame)
        depth_frame = self.temporal.process(depth_frame)
        depth_frame = self.hole_filling.process(depth_frame)

        # 擷取影像
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        annotated = color_image.copy()

        # 畫 thermal 框（紅色）
        if self.thermal_rgb_pixel and self.thermal_bbox:
            tx, ty = self.thermal_rgb_pixel
            x1, y1, x2, y2 = self.thermal_bbox
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.circle(annotated, (tx, ty), 5, (0, 0, 255), -1)
            cv2.putText(annotated, 'Thermal', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # YOLO 偵測
        results = self.model.predict(source=color_image, show=False, conf=0.4)
        best_box = None
        max_area = 0
        for box in results[0].boxes:
            if int(box.cls[0]) != 0:  # 只取 fire 類別
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)
            if area > max_area:
                max_area = area
                best_box = (x1, y1, x2, y2)
            # 畫出 YOLO 框（綠色）
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, 'YOLO fire', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 沒找到 fire 不推論深度
        if best_box is None:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
            return

        # 深度推算邏輯
        x1, y1, x2, y2 = best_box
        cx = (x1 + x2) // 2
        by = min(y2 + 5, depth_image.shape[0] - 1)
        scale_x = depth_image.shape[1] / color_image.shape[1]
        cx_scaled = int(cx * scale_x)

        min_depth = float('inf')
        best_y = by
        for y in range(by, min(by + 30, depth_image.shape[0]), 2):
            d = depth_image[y, cx_scaled] * self.depth_scale
            if 0 < d < min_depth:
                min_depth = d
                best_y = y

        if min_depth == float('inf'):
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
            return

        compensated = min_depth + 0.4
        intrin = color_frame.profile.as_video_stream_profile().intrinsics
        X, Y, Z = rs.rs2_deproject_pixel_to_point(intrin, [cx, best_y], min_depth)
        X_cam, Y_cam, Z_cam = Z, -X, -Y

        cv2.putText(annotated, f"Depth: {compensated:.2f}m", (x1, y1 - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 發布資料
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
        self.depth_pub.publish(Float32(data=compensated))
        self.pos_pub.publish(Point(x=X_cam, y=Y_cam, z=Z_cam))

        # 發布 Marker
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
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        # 畫 YOLO 框於深度圖
        colorized_depth = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
        x1_d = int(x1 * (depth_image.shape[1] / color_image.shape[1]))
        y1_d = int(y1 * (depth_image.shape[0] / color_image.shape[0]))
        x2_d = int(x2 * (depth_image.shape[1] / color_image.shape[1]))
        y2_d = int(y2 * (depth_image.shape[0] / color_image.shape[0]))
        cv2.rectangle(colorized_depth, (x1_d, y1_d), (x2_d, y2_d), (0, 255, 0), 2)
        self.depth_colormap_pub.publish(self.bridge.cv2_to_imgmsg(colorized_depth, 'bgr8'))

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
