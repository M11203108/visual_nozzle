#!/usr/bin/env python3
import math, time
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator

class AlignmentStepper(Node):
    def __init__(self):
        super().__init__('alignment_stepper')

        # ===== 參數 =====
        self.declare_parameter('yaw_topic', '/fire_yaw_error')       # 角度(度)
        self.declare_parameter('deadband_deg', 0.5)                  # 視為對正誤差
        self.declare_parameter('max_step_deg', 20.0)                 # 每步最大旋轉
        self.declare_parameter('min_step_deg', 1.0)                  # 小於此值就視為完成（防抖）
        self.declare_parameter('required_zero_cycles', 3)            # 連續幾次0才算穩
        self.declare_parameter('time_allowance_sec', 10)             # 每步動作時間（int）
        self.declare_parameter('post_move_min_sec', 0.3)             # 旋轉完至少等待秒數
        self.declare_parameter('max_settle_sec', 2.0)                # 最多等待穩定秒數（超時就繼續）
        self.declare_parameter('input_bias_deg', 0.0)                # 對輸入角度的偏移校正
        self.declare_parameter('stale_timeout_sec', 1.0)             # 多久沒更新就當舊資料

        # ⭐ 穩定條件
        self.declare_parameter('odom_topic', '/odom')                # 里程計 topic
        self.declare_parameter('angvel_thresh', 0.02)                # |ωz| < 0.02 rad/s 視為靜止
        self.declare_parameter('stationary_hold_sec', 0.3)           # 靜止需連續維持秒數
        self.declare_parameter('stability_deg', 0.7)                 # 最近視窗角度 max-min < 0.7°
        self.declare_parameter('stability_window', 8)                # 角度穩定視窗筆數

        # 讀參數
        self.yaw_topic = self.get_parameter('yaw_topic').value
        self.deadband_deg = float(self.get_parameter('deadband_deg').value)
        self.max_step_deg = float(self.get_parameter('max_step_deg').value)
        self.min_step_deg = float(self.get_parameter('min_step_deg').value)
        self.required_zero_cycles = int(self.get_parameter('required_zero_cycles').value)
        self.time_allowance_sec = int(self.get_parameter('time_allowance_sec').value)
        self.post_move_min_sec = float(self.get_parameter('post_move_min_sec').value)
        self.max_settle_sec = float(self.get_parameter('max_settle_sec').value)
        self.input_bias_deg = float(self.get_parameter('input_bias_deg').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.angvel_thresh = float(self.get_parameter('angvel_thresh').value)
        self.stationary_hold_sec = float(self.get_parameter('stationary_hold_sec').value)
        self.stability_deg = float(self.get_parameter('stability_deg').value)
        self.stability_window = int(self.get_parameter('stability_window').value)

        # ===== 狀態 =====
        self.yaw_raw = None
        self.yaw_used = None
        self.last_rx_time = 0.0
        self.zero_cnt = 0

        self.angvel = None
        self.last_below_thresh = None
        self.yaw_window = deque(maxlen=self.stability_window)

        # ===== Nav2 =====
        self.nav = BasicNavigator()
        self.get_logger().info('⏳ 等待 Nav2 Active …')
        self.nav.waitUntilNav2Active()
        self.get_logger().info('✅ Nav2 Active')

        # 訂閱
        self.create_subscription(Float32, self.yaw_topic, self._yaw_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

    # ---- callbacks ----
    def _yaw_cb(self, msg: Float32):
        self.yaw_raw = float(msg.data)
        self.yaw_used = self.yaw_raw + self.input_bias_deg
        self.last_rx_time = time.monotonic()
        self.yaw_window.append(self.yaw_used)
        self.get_logger().info(
            f"[RX] yaw_raw={self.yaw_raw:.2f}°, bias={self.input_bias_deg:+.2f}° -> yaw_used={self.yaw_used:.2f}°"
        )

    def _odom_cb(self, msg: Odometry):
        self.angvel = float(msg.twist.twist.angular.z)
        now = time.monotonic()
        if abs(self.angvel) < self.angvel_thresh:
            # 記錄從何時開始「角速度低於門檻」
            if self.last_below_thresh is None:
                self.last_below_thresh = now
        else:
            self.last_below_thresh = None

    # ---- 穩定等待 ----
    def wait_until_stable(self):
        """
        旋轉完成後進入：至少等待 post_move_min_sec，
        之後直到同時滿足：
          - 角速度 |ωz| < angvel_thresh 並持續 stationary_hold_sec
          - 角度串流在最近 stability_window 筆內 max-min < stability_deg
        或超過 max_settle_sec 就放行。
        """
        t0 = time.monotonic()
        # 先等最小時間
        while rclpy.ok() and (time.monotonic() - t0) < self.post_move_min_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

        while rclpy.ok():
            elapsed = time.monotonic() - t0
            # 條件1：角速度門檻
            ok_angvel = self.last_below_thresh is not None \
                        and (time.monotonic() - self.last_below_thresh) >= self.stationary_hold_sec

            # 條件2：角度視窗穩定
            if len(self.yaw_window) >= max(3, self.stability_window // 2):
                y_min = min(self.yaw_window)
                y_max = max(self.yaw_window)
                ok_yaw = (y_max - y_min) <= self.stability_deg
            else:
                ok_yaw = False

            if ok_angvel and ok_yaw:
                self.get_logger().info(
                    f"🟢 穩定就緒：|ωz|<{self.angvel_thresh}rad/s 持續≥{self.stationary_hold_sec}s 且 "
                    f"yaw窗口Δ={y_max - y_min:.2f}° ≤ {self.stability_deg}°"
                )
                return

            if elapsed >= self.max_settle_sec:
                self.get_logger().warn(
                    f"⏱️ 穩定等待超時 {elapsed:.1f}s，放行（可能感測雜訊或里程計無法判定靜止）"
                )
                return

            rclpy.spin_once(self, timeout_sec=0.05)

    # ---- 主流程 ----
    def run(self):
        self.get_logger().info(f'🧭 對正開始：監看 {self.yaw_topic}，用 BasicNavigator.spin() 逐步旋轉')

        # 等第一筆角度
        while rclpy.ok() and self.yaw_used is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        while rclpy.ok():
            # 取得誤差（允許沿用上一筆）
            now = time.monotonic()
            age = now - self.last_rx_time
            err = float(self.yaw_used)

            if age > self.stale_timeout_sec:
                self.get_logger().warn(f"[STALE] {age:.2f}s 未收到新角度，沿用 {err:.2f}°")

            # 在 deadband 內就累計，連續達標即完成
            if abs(err) <= self.deadband_deg:
                self.zero_cnt += 1
                self.get_logger().info(
                    f"✅ 在 deadband 內 {err:.2f}° "
                    f"({self.zero_cnt}/{self.required_zero_cycles})"
                )
                if self.zero_cnt >= self.required_zero_cycles:
                    self.get_logger().info("🎯 對正完成，退出節點")
                    break

                # 還沒達標：等下一筆角度
                t_end = time.monotonic() + 0.2
                while rclpy.ok() and time.monotonic() < t_end:
                    rclpy.spin_once(self, timeout_sec=0.05)
                continue

            # 需要旋轉 → 歸零計數
            self.zero_cnt = 0

            # 計算這一步旋轉
            step_deg = max(-self.max_step_deg, min(self.max_step_deg, err))
            if abs(step_deg) < self.min_step_deg:
                self.get_logger().info(
                    f"🎯 誤差 {err:.2f}° < 最小步長 {self.min_step_deg:.2f}°，視為完成"
                )
                break

            step_rad = math.radians(step_deg)
            self.get_logger().info(
                f"[PLAN] curr_err={err:.2f}° -> cmd={step_deg:.2f}° ({step_rad:.3f} rad)"
            )

            # 下 /spin，並等待 BasicNavigator 回報完成
            self.get_logger().info(
                f"[SEND] nav.spin(spin_dist={step_rad:.3f} rad, time_allowance={self.time_allowance_sec}s)"
            )
            self.nav.spin(spin_dist=step_rad, time_allowance=self.time_allowance_sec)
            while rclpy.ok() and not self.nav.isTaskComplete():
                rclpy.spin_once(self.nav, timeout_sec=0.1)
                rclpy.spin_once(self, timeout_sec=0.0)

            self.get_logger().info("↺ 這一步旋轉完成，進入穩定檢查…")
            self.wait_until_stable()   # ⭐ 這裡會「確定穩定」後才離開

        rclpy.shutdown()

def main():
    rclpy.init()
    node = AlignmentStepper()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
