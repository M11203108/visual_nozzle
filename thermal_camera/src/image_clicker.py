import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageClicker(Node):
    def __init__(self):
        super().__init__('image_clicker')
        self.bridge = CvBridge()
        self.rs_image = None
        self.ir_image = None
        self.rs_points = []
        self.ir_points = []

        # 訂閱 RealSense 與 熱像儀 的影像
        self.rs_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rs_callback, 10)
        self.ir_sub = self.create_subscription(Image, '/ir_camera/image', self.ir_callback, 10)

        cv2.namedWindow('RealSense Image')
        cv2.setMouseCallback('RealSense Image', self.rs_mouse_callback)

        cv2.namedWindow('Thermal Image')
        cv2.setMouseCallback('Thermal Image', self.ir_mouse_callback)

    def rs_callback(self, msg):
        self.rs_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def ir_callback(self, msg):
        self.ir_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

    def rs_mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.rs_points.append((x, y))
            print(f"RealSense 點擊座標: {x}, {y}")

    def ir_mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.ir_points.append((x, y))
            print(f"Thermal 點擊座標: {x}, {y}")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            if self.rs_image is not None:
                rs_display = self.rs_image.copy()
                for point in self.rs_points:
                    cv2.circle(rs_display, point, 5, (0, 0, 255), -1)
                cv2.imshow('RealSense Image', rs_display)

            if self.ir_image is not None:
                ir_display = cv2.applyColorMap(self.ir_image, cv2.COLORMAP_JET)
                for point in self.ir_points:
                    cv2.circle(ir_display, point, 5, (0, 0, 255), -1)
                cv2.imshow('Thermal Image', ir_display)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 儲存對應點
        np.save('rs_points.npy', self.rs_points)
        np.save('ir_points.npy', self.ir_points)

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ImageClicker()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
