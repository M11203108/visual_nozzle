#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import cv2
import pyrealsense2 as rs
import os

class ThermalFusionNode(Node):
    def __init__(self):
        super().__init__('thermal_fusion_node')
        self.bridge = CvBridge()

        # 載入 Homography
        self.H = np.load('/home/robot/newjasmine_ws/src/thermal_camera/scripts/H_avg.npy')
        self.get_logger().info("✅ 已載入 H_avg.npy")

        # 初始化 RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.intrinsics = None
        self.depth_scale = None

        # 訂閱熱像儀最大溫度點
        self.create_subscription(Float32MultiArray, '/thermal_max_temp', self.thermal_callback, 10)

        # 訂閱熱像儀畫面
        self.create_subscription(Image, '/ir_camera/image', self.thermal_image_callback, 10)
        self.thermal_img = None

        # 發佈火源空間座標
        self.fire_pub = self.create_publisher(Point, '/thermal_fire_position', 10)

        # 建立視窗
        for name in [
            "Overlay (Thermal + RGB)",
            "Thermal Warped",
            "Thermal Raw (gray)",
            "RealSense RGB",
            "RealSense Depth"
        ]:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)

    def thermal_image_callback(self, msg):
        self.thermal_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

    def thermal_callback(self, msg):
        if len(msg.data) < 7:
            self.get_logger().warn("⚠️ 熱像儀資料不完整，需包含 [tx, ty, temp, xmin, xmax, ymin, ymax]")
            return

        tx, ty, temp, xmin, xmax, ymin, ymax = msg.data
        tx, ty = int(tx), int(ty)

        # === 取 RealSense 畫面 ===
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame or self.thermal_img is None:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        if self.intrinsics is None:
            self.intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
            self.depth_scale = depth_frame.get_units()

        # === Homography 對應熱像儀點到 RGB 上 ===
        pt_src = np.array([[tx, ty]], dtype='float32')
        pt_dst = cv2.perspectiveTransform(pt_src[None], self.H)[0][0]
        x_rs, y_rs = int(pt_dst[0]), int(pt_dst[1])

        if not (0 <= x_rs < depth_image.shape[1] and 0 <= y_rs < depth_image.shape[0]):
            self.get_logger().warn("🔥 對應點超出範圍")
            return

        depth_val = depth_image[y_rs, x_rs] * self.depth_scale
        if depth_val == 0:
            self.get_logger().warn("📭 對應點深度為 0，跳過")
            return

        # === 計算對應的 3D 座標 ===
        X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [x_rs, y_rs], depth_val)

        fire_point = Point()
        fire_point.x = Z
        fire_point.y = -X
        fire_point.z = -Y
        self.fire_pub.publish(fire_point)
        self.get_logger().info(f'🔥 熱像火源深度座標: ({fire_point.x:.2f}, {fire_point.y:.2f}, {fire_point.z:.2f}) m')

        # === 對齊熱像儀影像 + 上色 ===
        aligned_thermal = cv2.warpPerspective(self.thermal_img, self.H, 
                                            (color_image.shape[1], color_image.shape[0]))
        thermal_color = cv2.applyColorMap(aligned_thermal, cv2.COLORMAP_JET)

        # === 疊加畫面 ===
        overlay = cv2.addWeighted(color_image, 0.6, thermal_color, 0.4, 0)
        cv2.circle(overlay, (x_rs, y_rs), 8, (0, 0, 255), 2)
        cv2.putText(overlay, f"{temp:.1f}C", (x_rs + 10, y_rs - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # # 🔲 加上紅框（熱區）
        # cv2.rectangle(overlay,
        #             (int(xmin), int(ymin)),
        #             (int(xmax), int(ymax)),
        #             (0, 0, 255), 2)

        # 🔲 加上紅框（熱區）
        x1, y1 = int(msg.data[3]), int(msg.data[4])
        x2, y2 = int(msg.data[5]), int(msg.data[6])

        # 套用 Homography 到兩個角點
        p1 = cv2.perspectiveTransform(np.array([[[x1, y1]]], dtype=np.float32), self.H)[0][0]
        p2 = cv2.perspectiveTransform(np.array([[[x2, y2]]], dtype=np.float32), self.H)[0][0]
        pt1 = (int(p1[0]), int(p1[1]))
        pt2 = (int(p2[0]), int(p2[1]))

        cv2.rectangle(overlay, pt1, pt2, (0, 0, 255), 2)


        # === 深度圖彩色化 ===
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # === 顯示所有畫面 ===
        cv2.imshow("Overlay (Thermal + RGB)", overlay)
        cv2.imshow("Thermal Warped", thermal_color)
        cv2.imshow("Thermal Raw (gray)", self.thermal_img)
        cv2.imshow("RealSense RGB", color_image)
        cv2.imshow("RealSense Depth", depth_colormap)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
