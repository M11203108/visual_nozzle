#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2

class ThermalToRGBPixelNode(Node):
    def __init__(self):
        super().__init__('thermal_to_rgb_pixel_node')

        # 載入 Homography
        self.H = np.load('/home/robot/newjasmine_ws/src/thermal_camera/scripts/H_avg.npy')
        self.get_logger().info("✅ 已載入 H_avg.npy")

        # 訂閱熱傳像最大溫度點（tx, ty, temp, xmin, xmax, ymin, ymax）
        self.create_subscription(Float32MultiArray, '/thermal_max_temp', self.thermal_callback, 10)

        # 發布轉換後的 RGB 像素座標 + 框
        self.pixel_pub = self.create_publisher(Float32MultiArray, '/thermal_to_rgb_pixel', 10)

        # 預設 RGB 解像度 (RealSense)
        self.rgb_width = 1280
        self.rgb_height = 720

    def thermal_callback(self, msg):
        if len(msg.data) < 7:
            self.get_logger().warn("⚠️ 熱傳像資料不足 (tx, ty, xmin, xmax, ymin, ymax)")
            return

        tx, ty = msg.data[0], msg.data[1]
        xmin, xmax = msg.data[3], msg.data[4]
        ymin, ymax = msg.data[5], msg.data[6]

        # 轉換熱點座標
        pt_src = np.array([[tx, ty]], dtype='float32')
        pt_dst = cv2.perspectiveTransform(pt_src[None], self.H)[0][0]
        x_rs, y_rs = int(pt_dst[0]), int(pt_dst[1])

        if not (0 <= x_rs < self.rgb_width and 0 <= y_rs < self.rgb_height):
            self.get_logger().warn(f"🔥 對應點超出範圍: ({x_rs}, {y_rs})")
            return

        # 框的上左與下右點轉換
        pt_tl = np.array([[xmin, ymin]], dtype='float32')
        pt_br = np.array([[xmax, ymax]], dtype='float32')
        pt_tl_rs = cv2.perspectiveTransform(pt_tl[None], self.H)[0][0]
        pt_br_rs = cv2.perspectiveTransform(pt_br[None], self.H)[0][0]

        x1, y1 = int(np.clip(pt_tl_rs[0], 0, self.rgb_width - 1)), int(np.clip(pt_tl_rs[1], 0, self.rgb_height - 1))
        x2, y2 = int(np.clip(pt_br_rs[0], 0, self.rgb_width - 1)), int(np.clip(pt_br_rs[1], 0, self.rgb_height - 1))

        # 發布 [x, y, x1, y1, x2, y2]
        pixel_msg = Float32MultiArray()
        pixel_msg.data = [float(x_rs), float(y_rs), float(x1), float(y1), float(x2), float(y2)]
        self.pixel_pub.publish(pixel_msg)

        self.get_logger().info(f"📍 RGB 熱點: ({x_rs}, {y_rs}) 框: ({x1},{y1})~({x2},{y2})")


def main(args=None):
    rclpy.init(args=args)
    node = ThermalToRGBPixelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
