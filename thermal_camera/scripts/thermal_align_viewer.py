#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import os

class ThermalAlignViewer(Node):
    def __init__(self):
        super().__init__('thermal_align_viewer')
        self.bridge = CvBridge()

        # 載入 Homography
        self.H = np.load('/home/robot/newjasmine_ws/src/thermal_camera/scripts/H_avg.npy')
        print("✅ 已載入 Homography 矩陣 H_avg.npy")

        # 訂閱熱像儀與 RealSense 彩色影像
        self.create_subscription(Image, '/ir_camera/image', self.thermal_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.realsense_callback, 10)

        self.thermal_img = None
        self.realsense_img = None

    def thermal_callback(self, msg):
        self.thermal_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')  # 假設為單通道熱像
        self.show_combined()

    def realsense_callback(self, msg):
        self.realsense_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.show_combined()

    def show_combined(self):
        if self.thermal_img is None or self.realsense_img is None:
            return

        # 校正熱像儀影像（對齊至 RealSense）
        aligned_thermal = cv2.warpPerspective(self.thermal_img, self.H, 
                                              (self.realsense_img.shape[1], self.realsense_img.shape[0]))

        # 轉成彩色（便於合成）
        thermal_color = cv2.applyColorMap(aligned_thermal, cv2.COLORMAP_JET)

        # 疊加熱像到 realsense 畫面
        overlay = cv2.addWeighted(self.realsense_img, 0.6, thermal_color, 0.4, 0)

        
        cv2.imshow("Thermal warped", aligned_thermal)

        cv2.imshow("Original Thermal", self.thermal_img)

        cv2.imshow("RealSense", self.realsense_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ThermalAlignViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
