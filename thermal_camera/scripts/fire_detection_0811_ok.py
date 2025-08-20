#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String
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

        self.image_pub = self.create_publisher(Image, 'fire_detection_image', 10)
        self.depth_colormap_pub = self.create_publisher(Image, 'depth_colormap', 10)
        self.depth_pub = self.create_publisher(Float32, 'fire_depth', 10)
        self.pos_pub = self.create_publisher(Point, 'fire_position', 10)
        self.marker_pub = self.create_publisher(Marker, 'fire_marker', 10)
        self.info_pub = self.create_publisher(String, 'fire_info', 10)

        self.size_pub = self.create_publisher(Float32MultiArray, 'fire_size', 10)

        self.thermal_pixel_sub = self.create_subscription(
            Float32MultiArray, '/thermal_to_rgb_pixel', self.thermal_pixel_callback, 10)
        self.thermal_rgb_pixel = None
        self.thermal_bbox = None

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('Fire Detection Node 啟動')

    def thermal_pixel_callback(self, msg):
        if len(msg.data) >= 6:
            self.thermal_rgb_pixel = (int(msg.data[0]), int(msg.data[1]))
            self.thermal_bbox = tuple(map(int, msg.data[2:6]))
            self.get_logger().info(f"Thermal 框與點接收: pt=({self.thermal_rgb_pixel}), box={self.thermal_bbox}")

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        depth_frame = self.decimation.process(depth_frame)
        depth_frame = self.spatial.process(depth_frame)
        depth_frame = self.temporal.process(depth_frame)
        depth_frame = self.hole_filling.process(depth_frame)

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        annotated = color_image.copy()

        source = 'YOLO'
        center_px = None
        bbox = None

        if self.thermal_rgb_pixel and self.thermal_bbox:
            tx, ty = self.thermal_rgb_pixel
            x1, y1, x2, y2 = self.thermal_bbox
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.circle(annotated, (tx, ty), 5, (0, 0, 255), -1)
            center_px = (tx, ty)
            bbox = (x1, y1, x2, y2)
            source = 'Thermal'

        results = self.model.predict(source=color_image, show=False, conf=0.4)
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
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, 'YOLO fire', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            center_px = (cx, cy)
            bbox = best_box
            source = 'YOLO'

        if center_px is None:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
            return

        scale_x = depth_image.shape[1] / color_image.shape[1]
        scale_y = depth_image.shape[0] / color_image.shape[0]
        dx = int(center_px[0] * scale_x)
        dy = int(center_px[1] * scale_y)

        half_window = 5
        window = depth_image[max(0, dy - half_window):min(depth_image.shape[0], dy + half_window + 1),
                              max(0, dx - half_window):min(depth_image.shape[1], dx + half_window + 1)]
        valid = window[window > 0] * self.depth_scale
        if len(valid) == 0:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
            return

        avg_depth = float(np.mean(valid))
        compensated = avg_depth + 0.4

        intrin = color_frame.profile.as_video_stream_profile().intrinsics
        X, Y, Z = rs.rs2_deproject_pixel_to_point(intrin, [center_px[0], center_px[1]], avg_depth)
        X_cam, Y_cam, Z_cam = Z, -X, -Y

        # X_robot = X_cam
        # Y_robot = Y_cam + 0.12
        # Z_robot = Z_cam - 0.975

        if source == 'Thermal':
            Y_robot = Y_cam + 0.11
            Z_robot = Z_cam + 0.98
        else:  # YOLO → RealSense
            Y_robot = Y_cam + 0.12
            Z_robot = Z_cam + 0.975

        X_robot = X_cam


        # fire_w = fire_l = 0.02 * (bbox[2] - bbox[0])
        # fire_h = 0.3 * fire_w

        # === Fire Size 推估 ===
        pixel_to_meter_yolo = 0.002  # 每 pixel 對應約 2mm（YOLO RGB）
        pixel_to_meter_thermal = 0.0005  # 每 pixel 對應約 5mm（Thermal，可依實測微調）

        if source == 'Thermal':
            pixel_w = self.thermal_bbox[2] - self.thermal_bbox[0]
            pixel_h = self.thermal_bbox[3] - self.thermal_bbox[1]
            fire_w = fire_l = pixel_w * pixel_to_meter_thermal
            fire_h = pixel_h * pixel_to_meter_thermal
        else:  # YOLO
            pixel_w = bbox[2] - bbox[0]
            pixel_h = bbox[3] - bbox[1]
            fire_w = fire_l = pixel_w * pixel_to_meter_yolo
            fire_h = pixel_h * pixel_to_meter_yolo


        info_msg = f"fire_center: ({X_robot:.2f}, {Y_robot:.2f}, {Z_robot:.2f}), fire_size: ({fire_l:.2f}, {fire_w:.2f}, {fire_h:.2f})"
        self.info_pub.publish(String(data=info_msg))

        size_msg = Float32MultiArray()
        size_msg.data = [fire_l, fire_w, fire_h]
        self.size_pub.publish(size_msg)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
        self.depth_pub.publish(Float32(data=compensated))
        self.pos_pub.publish(Point(x=X_robot, y=Y_robot, z=Z_robot))

        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fire'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = X_robot
        marker.pose.position.y = Y_robot
        marker.pose.position.z = Z_robot
        marker.pose.orientation.w = 1.0
        marker.scale.x = fire_l
        marker.scale.y = fire_w
        marker.scale.z = fire_h
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        colorized_depth = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
        x1_d = int(bbox[0] * scale_x)
        y1_d = int(bbox[1] * scale_y)
        x2_d = int(bbox[2] * scale_x)
        y2_d = int(bbox[3] * scale_y)
        cv2.rectangle(colorized_depth, (x1_d, y1_d), (x2_d, y2_d), (0, 0, 255) if source == 'Thermal' else (0, 255, 0), 2)
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
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
