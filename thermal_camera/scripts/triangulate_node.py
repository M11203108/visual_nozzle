#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import numpy as np
import cv2

class TriangulateNode(Node):
    def __init__(self):
        super().__init__('triangulate_node')

        self.sub = self.create_subscription(Float32MultiArray, '/thermal_max_temp', self.callback, 10)
        self.pub = self.create_publisher(Point, '/fire_position', 10)

        # 載入標定結果
        calib_file = '/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046/stereo_result.npz'
        data = np.load(calib_file)
        self.K1, self.D1 = data['K1'], data['D1']
        self.K2, self.D2 = data['K2'], data['D2']
        self.R, self.T = data['R'], data['T']
        self.P1 = self.K1 @ np.hstack((np.eye(3), np.zeros((3, 1))))
        self.P2 = self.K2 @ np.hstack((self.R, self.T))

    def triangulate_thermal_to_3d(self, thermal_px, realsense_px):
        pt1 = np.array(thermal_px, dtype=np.float32).reshape(2, 1)
        pt2 = np.array(realsense_px, dtype=np.float32).reshape(2, 1)
        point_4d = cv2.triangulatePoints(self.P1, self.P2, pt1, pt2)
        point_3d = point_4d[:3] / point_4d[3]
        return point_3d.flatten()

    def callback(self, msg):
        thermal_px = (msg.data[0], msg.data[1])
        # TODO: Replace with dynamic mapping from thermal to realsense image
        realsense_px = (325.0, 140.0)  # ★ 實際使用時應透過特徵對應或 homography 對應

        point3d = self.triangulate_thermal_to_3d(thermal_px, realsense_px)

        p = Point()
        p.x, p.y, p.z = point3d.tolist()
        self.pub.publish(p)

        self.get_logger().info(f"🔥 發佈火源 3D 座標: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = TriangulateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
