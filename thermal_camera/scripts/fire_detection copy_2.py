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
from geometry_msgs.msg import Point


class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        self.bridge = CvBridge()
        self.marker_pub = self.create_publisher(Marker, 'fire_marker', 10)

        # 載入 YOLO 模型
        base_path = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(base_path, 'best.pt')
        self.model = YOLO(model_path)

        # 初始化 RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)

        # 建立 ROS2 的 publisher
        self.image_pub = self.create_publisher(Image, 'fire_detection_image', 10)
        self.depth_pub = self.create_publisher(Float32, 'fire_depth', 10)
        self.pos_pub = self.create_publisher(Point, 'fire_position', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('🔥 Fire Detection Node 已啟動')

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())

        # 使用 YOLO 模型進行火焰辨識
        results = self.model.predict(source=color_image, show=False, conf=0.4)
        annotated_frame = results[0].plot()

        max_area = 0
        best_box = None

        # 找出面積最大的火源框
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)
            if area > max_area:
                max_area = area
                best_box = (x1, y1, x2, y2)

        if best_box:
            x1, y1, x2, y2 = best_box
            center_x = (x1 + x2) // 2
            bottom_y = min(y2 + 5, depth_frame.get_height() - 1)

            # 找出底部區域內的最小有效深度
            min_depth = float('inf')
            best_y = bottom_y
            for new_y in range(bottom_y, min(bottom_y + 30, depth_frame.get_height()), 2):
                depth = depth_frame.get_distance(center_x, new_y)
                if 0 < depth < min_depth:
                    min_depth = depth
                    best_y = new_y

            if min_depth != float('inf'):
                compensated_depth = min_depth + 0.4  # 深度補償
                cv2.putText(annotated_frame, f"Depth: {compensated_depth:.2f}m", (x1, y1 - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # 發布 fire_depth
                depth_msg = Float32()
                depth_msg.data = compensated_depth
                self.depth_pub.publish(depth_msg)

                # 計算 3D 空間座標 (X, Y, Z)
                intrin = color_frame.profile.as_video_stream_profile().intrinsics
                X, Y, Z = rs.rs2_deproject_pixel_to_point(intrin, [center_x, best_y], min_depth)

                

                # optical frame → camera_link 的轉換
                X_opt, Y_opt, Z_opt = X, Y, Z
                X_cam = Z_opt
                Y_cam = -X_opt
                Z_cam = -Y_opt

                # 發布 fire_position（給其他節點用）
                pos_msg = Point()
                pos_msg.x = X_cam
                pos_msg.y = Y_cam
                pos_msg.z = Z_cam
                self.pos_pub.publish(pos_msg)

                # 發布 Marker 給 RViz2 顯示
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


                self.get_logger().info(f'🔥 Fire Position: ({X:.2f}, {Y:.2f}, {Z:.2f}) m')

        # 發布影像
        image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.image_pub.publish(image_msg)

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
