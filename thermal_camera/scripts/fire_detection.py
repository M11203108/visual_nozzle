#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import numpy as np
import cv2
import pyrealsense2 as rs
import os
from ultralytics import YOLO

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        self.bridge = CvBridge()
        self.model = YOLO(os.path.join(os.path.dirname(__file__), 'best.pt'))

        # === RealSense 初始化 ===
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.pipeline.start(config)

        self.align = rs.align(rs.stream.color)
        self.decimation = rs.decimation_filter()
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        self.hole_filling = rs.hole_filling_filter()
        self.colorizer = rs.colorizer()
        self.depth_scale = self.pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()

        # === Pub/Sub ===
        self.image_pub = self.create_publisher(Image, 'fire_detection_image', 10)
        self.depth_colormap_pub = self.create_publisher(Image, 'depth_colormap', 10)
        self.depth_pub = self.create_publisher(Float32, 'fire_depth', 10)
        self.pos_pub = self.create_publisher(Point, 'fire_position', 10)
        self.marker_pub = self.create_publisher(Marker, 'fire_marker', 10)
        self.info_pub = self.create_publisher(String, 'fire_info', 10)
        self.size_pub = self.create_publisher(Float32MultiArray, 'fire_size', 10)

        self.thermal_pixel_sub = self.create_subscription(
            Float32MultiArray, '/thermal_to_rgb_pixel', self.thermal_pixel_callback, 10)
        self.thermal_rgb_pixel = None   # (tx, ty)
        self.thermal_bbox = None        # (x1, y1, x2, y2)

        # === 參數 ===
        self.yaw_error_pub = self.create_publisher(Float32, 'fire_yaw_error', 10)
        self.declare_parameter('yaw_deadband_deg', 3.0)
        self.yaw_deadband_deg = float(self.get_parameter('yaw_deadband_deg').value)

        # RealSense 相機相對 base_link 的位移（m）
        self.declare_parameter('rs_offset_x', 0.0)
        self.declare_parameter('rs_offset_y', 0.12)
        self.declare_parameter('rs_offset_z', 0.975)

        # Thermal 相機相對 base_link 的位移（m）
        self.declare_parameter('th_offset_x', 0.0)
        self.declare_parameter('th_offset_y', 0.11)
        self.declare_parameter('th_offset_z', 0.98)

        # 噴頭/底盤旋轉中心相對 base_link（m）
        self.declare_parameter('nozzle_offset_x', 0.0)
        self.declare_parameter('nozzle_offset_y', 0.0)

        # 讀參數
        self.rs_offset_x = float(self.get_parameter('rs_offset_x').value)
        self.rs_offset_y = float(self.get_parameter('rs_offset_y').value)
        self.rs_offset_z = float(self.get_parameter('rs_offset_z').value)
        self.th_offset_x = float(self.get_parameter('th_offset_x').value)
        self.th_offset_y = float(self.get_parameter('th_offset_y').value)
        self.th_offset_z = float(self.get_parameter('th_offset_z').value)
        self.nozzle_offset_x = float(self.get_parameter('nozzle_offset_x').value)
        self.nozzle_offset_y = float(self.get_parameter('nozzle_offset_y').value)

        # 邊界（機器人座標系, m）— 可依場地微調
        self.bound_x = (1.5, 4.5)
        self.bound_y = (-2.0, 2.0)
        self.bound_z = (0.0, 5.0)

        # 尺寸換算常數（依來源不同）
        self.pixel_to_meter_yolo = 0.002    # YOLO：RGB 像素轉實際尺寸
        self.pixel_to_meter_thermal = 0.0005 # Thermal：低解析像素轉實際尺寸

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('Fire Detection Node 啟動（雙候選→選擇→發布）')

    # ================= 工具函式 =================

    def thermal_pixel_callback(self, msg):
        if len(msg.data) >= 6:
            self.thermal_rgb_pixel = (int(msg.data[0]), int(msg.data[1]))
            self.thermal_bbox = tuple(map(int, msg.data[2:6]))
            # self.get_logger().info(f"Thermal 框與點接收: pt={self.thermal_rgb_pixel}, box={self.thermal_bbox}")

    def depth_at_pixel(self, depth_image, px, py, half_window=5):
        """回傳該像素附近的平均深度（米），無效回傳 None。"""
        h, w = depth_image.shape[:2]
        x1 = max(0, px - half_window)
        y1 = max(0, py - half_window)
        x2 = min(w, px + half_window + 1)
        y2 = min(h, py + half_window + 1)
        window = depth_image[y1:y2, x1:x2]
        valid = window[window > 0] * self.depth_scale
        if valid.size == 0:
            return None
        return float(np.mean(valid))

    def deproject_to_robot(self, intrin, center_px, depth_m, source):
        """像素+深度 → 相機座標 → 機器人座標（含來源相機位移補償）"""
        X, Y, Z = rs.rs2_deproject_pixel_to_point(intrin, [center_px[0], center_px[1]], depth_m)
        # 依你原本的座標換軸（camera系→REP-103機器人系）
        X_cam, Y_cam, Z_cam = Z, -X, -Y

        if source == 'Thermal':
            return (
                X_cam + self.th_offset_x,
                Y_cam + self.th_offset_y,
                Z_cam + self.th_offset_z
            )
        else:  # 'YOLO' → RealSense
            return (
                X_cam + self.rs_offset_x,
                Y_cam + self.rs_offset_y,
                Z_cam + self.rs_offset_z
            )

    def is_out_of_bounds(self, X, Y, Z):
        bx, by, bz = self.bound_x, self.bound_y, self.bound_z
        return not (bx[0] <= X <= bx[1] and by[0] <= Y <= by[1] and bz[0] <= Z <= bz[1])

    def build_candidate(self, name, center_px, bbox, intrin, color_shape, depth_image):
        """組裝候選資料：深度、機器人座標、像素尺寸、估測尺寸。回傳 dict 或 None。"""
        if center_px is None or bbox is None:
            return None

        # 深度採樣位置（depth 與 color 對齊後等比例縮放）
        scale_x = depth_image.shape[1] / color_shape[1]
        scale_y = depth_image.shape[0] / color_shape[0]
        dx = int(center_px[0] * scale_x)
        dy = int(center_px[1] * scale_y)

        depth_m = self.depth_at_pixel(depth_image, dx, dy, half_window=5)
        if depth_m is None:
            return None

        Xr, Yr, Zr = self.deproject_to_robot(intrin, center_px, depth_m, name)

        # 尺寸估測（依來源使用不同 pixel→meter）
        px_w = max(1, bbox[2] - bbox[0])
        px_h = max(1, bbox[3] - bbox[1])
        if name == 'Thermal':
            s = self.pixel_to_meter_thermal
        else:
            s = self.pixel_to_meter_yolo
        fire_w = px_w * s
        fire_h = px_h * s
        fire_l = fire_w  # 先以寬度近似長度

        return {
            'source': name,
            'center_px': center_px,
            'bbox': bbox,
            'depth_m': depth_m,
            'X': Xr, 'Y': Yr, 'Z': Zr,
            'px_w': px_w, 'px_h': px_h,
            'fire_w': fire_w, 'fire_h': fire_h, 'fire_l': fire_l,
            'scale_x': scale_x, 'scale_y': scale_y
        }

    # ================= 主流程 =================

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        # 濾波
        depth_frame = self.decimation.process(depth_frame)
        depth_frame = self.spatial.process(depth_frame)
        depth_frame = self.temporal.process(depth_frame)
        depth_frame = self.hole_filling.process(depth_frame)

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        annotated = color_image.copy()
        intrin = color_frame.profile.as_video_stream_profile().intrinsics

        # ====== 1) 準備兩個候選 ======

        # (a) Thermal 候選（若有）
        thermal_cand = None
        if self.thermal_rgb_pixel and self.thermal_bbox:
            tx, ty = self.thermal_rgb_pixel
            tb = self.thermal_bbox
            # 畫 Thermal（紅）
            cv2.rectangle(annotated, (tb[0], tb[1]), (tb[2], tb[3]), (0, 0, 255), 2)
            cv2.putText(annotated, 'Thermal', (tb[0], tb[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            thermal_cand = self.build_candidate(
                'Thermal', (tx, ty), tb, intrin, color_image.shape, depth_image
            )

        # (b) YOLO 候選（取最大框）
        yolo_cand = None
        results = self.model.predict(source=color_image, show=False, conf=0.4)
        best_box, max_area = None, 0
        for box in results[0].boxes:
            if int(box.cls[0]) != 0:  # 只要 fire 類別
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = max(1, (x2 - x1) * (y2 - y1))
            if area > max_area:
                max_area = area
                best_box = (x1, y1, x2, y2)
            # 畫 YOLO（綠）
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, 'YOLO', (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if best_box is not None:
            cx = (best_box[0] + best_box[2]) // 2
            cy = (best_box[1] + best_box[3]) // 2
            yolo_cand = self.build_candidate(
                'YOLO', (cx, cy), best_box, intrin, color_image.shape, depth_image
            )

        # ====== 2) 選擇最終候選 ======
        final = None
        if yolo_cand is None and thermal_cand is None:
            # 兩個都沒有 → 發布影像就好
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
            return
        elif yolo_cand is None:
            final = thermal_cand
        elif thermal_cand is None:
            final = yolo_cand
        else:
            # 都有 → 若 YOLO 超界，換 Thermal；否則用 YOLO
            if self.is_out_of_bounds(yolo_cand['X'], yolo_cand['Y'], yolo_cand['Z']):
                final = thermal_cand
            else:
                final = yolo_cand

        # 視覺標示最終中心（白點）
        if final is not None and final['center_px'] is not None:
            cv2.circle(annotated, final['center_px'], 6, (255, 255, 255), -1)

        # ====== 3) 計算對正角度（以噴頭旋轉中心為原點） ======
        X_rel = final['X'] - self.nozzle_offset_x
        Y_rel = final['Y'] - self.nozzle_offset_y
        yaw_error_rad = np.arctan2(Y_rel, X_rel)
        yaw_error_deg = float(np.degrees(yaw_error_rad))
        if abs(yaw_error_deg) <= self.yaw_deadband_deg:
            yaw_error_deg = 0.0
        self.yaw_error_pub.publish(Float32(data=yaw_error_deg))

        # ====== 4) 組合資訊並發布 ======
        # 深度可選擇做補償（例如加 0.4m），如需要解開下一行
        # compensated_depth = final['depth_m'] + 0.4
        compensated_depth = final['depth_m']

        info_msg = (f"source={final['source']}, yaw_err_deg={yaw_error_deg:.2f}, "
                    f"fire_center: ({final['X']:.2f}, {final['Y']:.2f}, {final['Z']:.2f}), "
                    f"fire_size: ({final['fire_l']:.2f}, {final['fire_w']:.2f}, {final['fire_h']:.2f})")
        self.info_pub.publish(String(data=info_msg))

        size_msg = Float32MultiArray()
        size_msg.data = [final['fire_l'], final['fire_w'], final['fire_h']]
        self.size_pub.publish(size_msg)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
        self.depth_pub.publish(Float32(data=compensated_depth))
        self.pos_pub.publish(Point(x=final['X'], y=final['Y'], z=final['Z']))

        # RViz Marker
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fire'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = final['X']
        marker.pose.position.y = final['Y']
        marker.pose.position.z = final['Z']
        marker.pose.orientation.w = 1.0
        marker.scale.x = final['fire_l']
        marker.scale.y = final['fire_w']
        marker.scale.z = final['fire_h']
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        # 深度彩圖：標示「最終選擇」的框
        colorized_depth = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
        x1_d = int(final['bbox'][0] * final['scale_x'])
        y1_d = int(final['bbox'][1] * final['scale_y'])
        x2_d = int(final['bbox'][2] * final['scale_x'])
        y2_d = int(final['bbox'][3] * final['scale_y'])
        cv2.rectangle(
            colorized_depth, (x1_d, y1_d), (x2_d, y2_d),
            (0, 0, 255) if final['source'] == 'Thermal' else (0, 255, 0), 2
        )
        self.depth_colormap_pub.publish(self.bridge.cv2_to_imgmsg(colorized_depth, 'bgr8'))

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
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
