#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import datetime

class DualCameraCalibration(Node):
    def __init__(self):
        super().__init__('dual_camera_calibration')
        self.bridge = CvBridge()

        self.sub_thermal = self.create_subscription(Image, '/ir_camera/image', self.thermal_callback, 10)
        self.sub_realsense = self.create_subscription(Image, '/camera/camera/color/image_raw', self.realsense_callback, 10)

        self.pattern_size = (3, 11)
        self.objp = self._generate_obj_points()

        # 建立儲存資料夾
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_root = os.path.join(os.path.dirname(__file__), f"dual_calib_{timestamp}")
        os.makedirs(self.save_root, exist_ok=True)

        self.thermal_data = {}
        self.realsense_data = {}
        self.img_count = 0

        self.last_thermal = None
        self.last_thermal_chessboard = None
        self.last_thermal_centers = None

        self.last_realsense = None
        self.last_realsense_chessboard = None
        self.last_realsense_centers = None

        # 建立 Trackbars
        self._init_trackbars("Trackbars_Thermal")
        self._init_trackbars("Trackbars_RealSense")

        self.get_logger().info("🟢 雙相機標定系統啟動完畢。按下 p 鍵儲存兩相機資料。")

    def _generate_obj_points(self):
        objp = []
        for c in range(self.pattern_size[0]):
            for r in range(self.pattern_size[1]):
                if r % 2 == 0:
                    objp.append([r, c*2, 0])
                else:
                    objp.append([r, c*2+1, 0])
        return np.array(objp, np.float32)

    def _init_trackbars(self, win):
        cv2.namedWindow(win)
        cv2.createTrackbar("Min Area", win, 10, 5000, lambda x: None)
        cv2.createTrackbar("Max Area", win, 5000, 5000, lambda x: None)
        cv2.createTrackbar("Min Circularity", win, 70, 100, lambda x: None)
        cv2.createTrackbar("Min Inertia", win, 50, 100, lambda x: None)
        cv2.createTrackbar("Min Convexity", win, 70, 100, lambda x: None)
        cv2.createTrackbar("Contrast", win, 100, 300, lambda x: None)
        cv2.createTrackbar("Brightness", win, 50, 100, lambda x: None)
        cv2.createTrackbar("THRESH", win, 0, 255, lambda x: None)

    def _get_blob_params(self, win):
        p = cv2.SimpleBlobDetector_Params()
        p.filterByArea = True
        p.minArea = max(cv2.getTrackbarPos("Min Area", win), 1)
        p.maxArea = max(cv2.getTrackbarPos("Max Area", win), 1)
        p.filterByCircularity = True
        p.minCircularity = cv2.getTrackbarPos("Min Circularity", win) / 100.0
        p.filterByInertia = True
        p.minInertiaRatio = cv2.getTrackbarPos("Min Inertia", win) / 100.0
        p.filterByConvexity = True
        p.minConvexity = cv2.getTrackbarPos("Min Convexity", win) / 100.0
        return p

    def detect_grid(self, gray, win):
        params = self._get_blob_params(win)
        detector = cv2.SimpleBlobDetector_create(params)
        ret, centers = cv2.findCirclesGrid(gray, self.pattern_size, None, 
                                           flags=cv2.CALIB_CB_ASYMMETRIC_GRID, blobDetector=detector)
        Chessboard_img = gray.copy()
        if ret:
            cv2.drawChessboardCorners(Chessboard_img, self.pattern_size, centers, ret)
        keypoints = detector.detect(gray)
        Circle_Grid = cv2.drawKeypoints(gray, keypoints, None, (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return ret, Circle_Grid, Chessboard_img, centers

    def thermal_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        alpha = cv2.getTrackbarPos("Contrast", "Trackbars_Thermal") / 100.0
        beta = cv2.getTrackbarPos("Brightness", "Trackbars_Thermal") - 50
        thresh = cv2.getTrackbarPos("THRESH", "Trackbars_Thermal")
        contrast = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
        _, thresh_img = cv2.threshold(contrast, thresh, 255, cv2.THRESH_TOZERO)

        ret, circle_img, chessboard_img, centers = self.detect_grid(thresh_img, "Trackbars_Thermal")
        self.last_thermal = img
        self.last_thermal_chessboard = chessboard_img
        self.last_thermal_centers = centers if ret else None

        cv2.imshow("thermal_origin", img)
        cv2.imshow("thermal_Circle", circle_img)
        cv2.imshow("thermal_Chess", chessboard_img)

        self.check_and_save()

    def realsense_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        alpha = cv2.getTrackbarPos("Contrast", "Trackbars_RealSense") / 100.0
        beta = cv2.getTrackbarPos("Brightness", "Trackbars_RealSense") - 50
        thresh = cv2.getTrackbarPos("THRESH", "Trackbars_RealSense")
        contrast = cv2.convertScaleAbs(gray, alpha=alpha, beta=beta)
        _, thresh_img = cv2.threshold(contrast, thresh, 255, cv2.THRESH_TOZERO)

        ret, circle_img, chessboard_img, centers = self.detect_grid(thresh_img, "Trackbars_RealSense")
        self.last_realsense = img
        self.last_realsense_chessboard = chessboard_img
        self.last_realsense_centers = centers if ret else None

        cv2.namedWindow("realsense_origin", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("realsense_origin", 640, 480)

        cv2.namedWindow("realsense_Circle", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("realsense_Circle", 640, 480)

        cv2.namedWindow("realsense_Chess", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("realsense_Chess", 640, 480)

        cv2.imshow("realsense_origin", img)
        cv2.imshow("realsense_Circle", circle_img)
        cv2.imshow("realsense_Chess", chessboard_img)

        self.check_and_save()

    def check_and_save(self):
        key = cv2.waitKey(1) & 0xFF
        if key == ord('p') and self.last_thermal_centers is not None and self.last_realsense_centers is not None:
            index = self.img_count
            cv2.imwrite(os.path.join(self.save_root, f"thermal_{index}.jpg"), self.last_thermal)
            cv2.imwrite(os.path.join(self.save_root, f"thermal_chess_{index}.jpg"), self.last_thermal_chessboard)
            cv2.imwrite(os.path.join(self.save_root, f"realsense_{index}.jpg"), self.last_realsense)
            cv2.imwrite(os.path.join(self.save_root, f"realsense_chess_{index}.jpg"), self.last_realsense_chessboard)

            np.savez(os.path.join(self.save_root, f"points_{index}.npz"),
                     obj_points=self.objp,
                     thermal_img_points=self.last_thermal_centers,
                     realsense_img_points=self.last_realsense_centers)

            print(f"✅ 儲存第 {index} 組影像與角點")
            self.img_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = DualCameraCalibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
