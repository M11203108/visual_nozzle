#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import pyrealsense2 as rs
import cv2
import numpy as np

class ThermalFusionNode(Node):
    def __init__(self):
        super().__init__('thermal_fusion_node')

        # === 1. 訂閱熱像儀最高溫點 ===
        self.sub = self.create_subscription(Float32MultiArray, '/thermal_max_temp', self.callback, 10)
        self.pub = self.create_publisher(Point, '/thermal_fire_position', 10)

        # === 2. 讀取 Homography 矩陣 ===
        calib_path = '/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046'
        # self.H = np.load(f"{calib_path}/homography.npy")
        self.H = np.load('/home/robot/newjasmine_ws/src/thermal_camera/scripts/H_avg.npy')

        # === 3. 初始化 RealSense ===
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # 對齊深度到 RGB
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # 取得相機內參
        profile = self.pipeline.get_active_profile()
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.fx, self.fy = intr.fx, intr.fy
        self.cx, self.cy = intr.ppx, intr.ppy

        self.get_logger().info("✅ ThermalFusionNode 啟動完成")

    def callback(self, msg):
        try:
            # === 1. 熱像儀座標 ===
            thermal_px = np.array([[msg.data[0], msg.data[1]]], dtype=np.float32)  # [[x, y]]

            # === 2. Homography 對應到 RealSense RGB ===
            rs_px = cv2.perspectiveTransform(np.array([thermal_px]), self.H)[0][0]
            u, v = int(rs_px[0]), int(rs_px[1])
            self.get_logger().info(f"🎯 投影熱點到 RealSense 像素: ({u}, {v})")

            # === 3. 抓 RealSense 畫面並對齊 ===
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            if not depth_frame:
                self.get_logger().warn("⚠️ 無法取得深度影像")
                return

            # === 4. 查該點的深度 ===
            depth = depth_frame.get_distance(u, v)
            if depth == 0.0:
                self.get_logger().warn("⚠️ 查無有效深度")
                return

            # === 5. 轉為空間座標 ===
            X = (u - self.cx) * depth / self.fx
            Y = (v - self.cy) * depth / self.fy
            Z = depth

            p = Point()
            p.x, p.y, p.z = X, Y, Z
            self.pub.publish(p)

            self.get_logger().info(f"📡 發佈 3D 火源位置: ({X:.2f}, {Y:.2f}, {Z:.2f})")
        except Exception as e:
            self.get_logger().error(f"❌ 錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ThermalFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
