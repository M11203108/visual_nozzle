#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import numpy as np
import cv2

class ThermalStereoFusionNode(Node):
    def __init__(self):
        super().__init__('thermal_stereo_fusion_node')
        self.sub = self.create_subscription(Float32MultiArray, '/thermal_max_temp', self.callback, 10)
        self.pub = self.create_publisher(Point, '/thermal_fire_position', 10)

        # Load calibration and homography data
        calib_path = '/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046/stereo_result.npz'
        homography_path = '/home/robot/newjasmine_ws/src/thermal_camera/scripts/H_avg.npy'

        data = np.load(calib_path)
        self.K1, self.K2 = data['K1'], data['K2']
        self.R, self.T = data['R'], data['T']
        self.H = np.load(homography_path)

        # Projection matrices
        self.P1 = self.K1 @ np.hstack((np.eye(3), np.zeros((3, 1))))
        self.P2 = self.K2 @ np.hstack((self.R, self.T))

    def triangulate(self, pt1, pt2):
        pt1 = np.array(pt1, dtype=np.float32).reshape(2, 1)
        pt2 = np.array(pt2, dtype=np.float32).reshape(2, 1)
        point_4d = cv2.triangulatePoints(self.P1, self.P2, pt1, pt2)
        point_3d = point_4d[:3] / point_4d[3]
        return point_3d.flatten()

    def callback(self, msg):
        tx, ty, temp = msg.data
        tx, ty = float(tx), float(ty)

        # Apply Homography to get realsense pixel
        pt_src = np.array([[tx, ty]], dtype='float32')[None]
        pt_dst = cv2.perspectiveTransform(pt_src, self.H)[0][0]
        x_rs, y_rs = float(pt_dst[0]), float(pt_dst[1])

        point3d = self.triangulate((tx, ty), (x_rs, y_rs))

        # Publish
        p = Point()
        p.x, p.y, p.z = point3d.tolist()
        self.pub.publish(p)
        self.get_logger().info(f"🔥 Triangulated 3D Fire Pos: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = ThermalStereoFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()