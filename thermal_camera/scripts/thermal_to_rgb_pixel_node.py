#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2

class ThermalToRGBPixelNode(Node):
    def __init__(self):
        super().__init__('thermal_to_rgb_pixel_node')

        # 載入 Homography 矩陣
        self.H = np.load('/home/robot/newjasmine_ws/src/thermal_camera/scripts/H_avg.npy')
        self.get_logger().info("✅ 已載入 H_avg.npy")

        # 訂閱熱像儀資訊：熱點 + 框座標
        self.create_subscription(Float32MultiArray, '/thermal_max_temp', self.thermal_callback, 10)

        # 發布轉換後的 RGB 像素座標與紅框
        self.pixel_pub = self.create_publisher(Float32MultiArray, '/thermal_to_rgb_pixel', 10)

        # RealSense RGB 尺寸
        self.rgb_width = 1280
        self.rgb_height = 720

    def thermal_callback(self, msg):
        if len(msg.data) < 7:
            self.get_logger().warn("⚠️ 熱像儀資料不足 (tx, ty, temp, xmin, xmax, ymin, ymax)")
            return

        # === 原始熱點與框座標 ===
        tx, ty = msg.data[0], msg.data[1]
        xmin, xmax = sorted([msg.data[3], msg.data[4]])
        ymin, ymax = sorted([msg.data[5], msg.data[6]])

        # === 轉換熱點 ===
        pt_src = np.array([[[tx, ty]]], dtype='float32')
        pt_dst = cv2.perspectiveTransform(pt_src, self.H)[0][0]
        x_rs, y_rs = int(pt_dst[0]), int(pt_dst[1])

        if not (0 <= x_rs < self.rgb_width and 0 <= y_rs < self.rgb_height):
            self.get_logger().warn(f"🔥 熱點 ({x_rs}, {y_rs}) 超出範圍")
            return

        # === 計算框中心與寬高 ===
        pt_tl = np.array([[[xmin, ymin]]], dtype='float32')
        pt_br = np.array([[[xmax, ymax]]], dtype='float32')
        pt_tl_rs = cv2.perspectiveTransform(pt_tl, self.H)[0][0]
        pt_br_rs = cv2.perspectiveTransform(pt_br, self.H)[0][0]

        cx = (pt_tl_rs[0] + pt_br_rs[0]) / 2
        cy = (pt_tl_rs[1] + pt_br_rs[1]) / 2
        w  = abs(pt_br_rs[0] - pt_tl_rs[0])
        h  = abs(pt_br_rs[1] - pt_tl_rs[1])

        # === 放大比例（可自訂）===
        scale_factor = 1.5  # 🔴 放大為原來的 1.5 倍
        new_w = w * scale_factor
        new_h = h * scale_factor

        x1 = int(np.clip(cx - new_w / 2, 0, self.rgb_width - 1))
        y1 = int(np.clip(cy - new_h / 2, 0, self.rgb_height - 1))
        x2 = int(np.clip(cx + new_w / 2, 0, self.rgb_width - 1))
        y2 = int(np.clip(cy + new_h / 2, 0, self.rgb_height - 1))

        # === 發布資料 ===
        pixel_msg = Float32MultiArray()
        pixel_msg.data = [float(x_rs), float(y_rs), float(x1), float(y1), float(x2), float(y2)]
        self.pixel_pub.publish(pixel_msg)

        self.get_logger().info(f"📍 RGB 熱點: ({x_rs}, {y_rs}) 放大框: ({x1},{y1})~({x2},{y2})")


def main(args=None):
    rclpy.init(args=args)
    node = ThermalToRGBPixelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # ✅ 加上這行做安全檢查
            rclpy.shutdown()


if __name__ == '__main__':
    main()
