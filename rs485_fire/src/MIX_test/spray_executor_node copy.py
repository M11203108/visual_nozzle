#!/usr/bin/env python3
# spray_executor_node.py  ────────────────────────────────────────────
#  /plan_status == finished → 執行掃描
#  掃描完成 / 中斷後：1 Hz × 5 s 連發 /spray_status=done
#  TEST_MODE=True → 只開 LED、30 秒自動結束
# ────────────────────────────────────────────────────────────────────
import os, csv, json, time, sys, threading
import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

# ====== ✨ 依需求切換這一行 ✨ =========================================
TEST_MODE = True          # True = 測試（30 s、自動結束） / False = 實機
# ====================================================================

# ── Motor / Modbus ---------------------------------------------------
#   測試模式依舊使用真馬達；若您沒有馬達可把下方兩行改成 Dummy 版本
from motor_control.motor1 import MotorControllerupdo  # 控制上下馬達
from motor_control.motor2 import MotorControllerLR   # 控制左右馬達
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException


UNIT            = 0x01
SENSOR_WATER    = 0x0004
COIL_BALL       = 0x0006      # 球閥
COIL_MOTOR      = 0x0003      # 泵浦
COIL_LED_L      = 0x0004
COIL_LED_R      = 0x0005

PKG_DIR = os.path.dirname(__file__)

# ---------- 輔助 ------------------------------------------------------
def _load_json(fname):
    with open(os.path.join(PKG_DIR, fname)) as f:
        return json.load(f)

def _load_traj(csv_name="trajectories/trajectory_angles.csv"):
    path = os.path.join(PKG_DIR, csv_name)
    with open(path) as f:
        return [(float(r['pitch_deg']), float(r['yaw_deg']))
                for r in csv.DictReader(f)]

def _segment(traj):
    seg, cur, buf = [], None, []
    for p, y in traj:
        if p != cur:
            if buf: seg.append((cur, buf))
            cur, buf = p, []
        buf.append(y)
    if buf: seg.append((cur, buf))
    return seg

# ---------- ROS2 節點 -------------------------------------------------
class SprayExecutor(Node):
    def __init__(self):
        super().__init__("spray_executor")
        self.get_logger().info("🟢 Waiting /plan_status == 'finished' ...")

        # QoS：確保晚加入仍能收到
        qos_transient = QoSProfile(
            depth      = 1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability =QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(String, "/plan_status",
                                 self._plan_cb, 10)
        self.status_pub = self.create_publisher(
            String, "/spray_status", qos_transient)

        # ── 硬體 --------------------------------------------------------
        self.modbus = ModbusClient(method='rtu', port='/dev/IOttyUSB',
                                   baudrate=9600, stopbits=2,
                                   bytesize=8, timeout=3)
        if not self.modbus.connect():
            self.get_logger().fatal("Modbus connect failed")
            sys.exit(1)

        self.up = MotorControllerupdo('/dev/updottyUSB', 9600, 0x00)
        self.lr = MotorControllerLR  ('/dev/LRttyUSB',  9600, 0x01)
        if not (self.up.connect() and self.lr.connect()):
            self.get_logger().fatal("Motor connect failed"); sys.exit(1)

        self.up_cfg = _load_json("calibration_updo.json")
        self.lr_cfg = _load_json("calibration_LR.json")
        self.traj   = _segment(_load_traj())
        self.running = False

    # ---------- /plan_status callback ---------------------------------
    def _plan_cb(self, msg: String):
        if msg.data.strip() != "finished" or self.running:
            return
        self.running = True
        threading.Thread(target=self._do_scan,
                         daemon=True).start()

    # ---------- 背景掃描 ----------------------------------------------
    def _do_scan(self):
        try:
            self._scan_once()
            self.get_logger().info("✅ Spray finished")
        except Exception as e:
            self.get_logger().error(f"⚠️ Spray aborted: {e}")
        finally:
            # 連續 5 s 送 /spray_status=done
            end = self.get_clock().now() + rclpy.duration.Duration(seconds=5)
            while self.get_clock().now() < end and rclpy.ok():
                self.status_pub.publish(String(data="done"))
                time.sleep(1.0)
            self._cleanup()

    # ---------- 主掃描流程（完全仿舊版邏輯） --------------------------
    def _scan_once(self):
        self._open_actuators()

        up0, lr0  = self.up_cfg["zero_encoder"], self.lr_cfg["zero_encoder"]
        up_min, up_max = self.up_cfg["down_limit_angle"], self.up_cfg["up_limit_angle"]
        lr_min, lr_max = self.lr_cfg["right_limit_angle"], self.lr_cfg["left_limit_angle"]

        start_t   = time.time()
        reverse   = False                    # 和舊版一樣：整圈交替

        while True:
            # ★ 這一行決定 pitch 順序  ---------------------- #
            segment_iter = self.traj if not reverse else reversed(self.traj)

            for pitch, yaws in segment_iter:

                # ---- TEST_MODE / 水位檢查 ------------------
                if TEST_MODE and time.time() - start_t >= 30:
                    self.get_logger().info("⏰ 30-s timeout (TEST_MODE)")
                    self._close_actuators()
                    return
                if (not TEST_MODE) and self._water_low():
                    self._close_actuators()
                    raise RuntimeError("water_low")

                # ---- Up-Down ------------------------------
                hw_pitch = 26.0 + pitch
                if not up_min <= hw_pitch <= up_max:
                    self.get_logger().warn(f"Pitch {hw_pitch:.2f}° out-of-range")
                    continue
                self.up.move_to_angle(up0, hw_pitch)
                

                # ---- Left-Right ---------------------------
                min_y, max_y = min(yaws), max(yaws)
                if not (lr_min <= min_y <= lr_max and lr_min <= max_y <= lr_max):
                    self.get_logger().warn("Yaw out-of-range");  continue

                if not reverse:
                    self.lr.move_to_angle(lr0, min_y)
                    self.lr.move_to_angle(lr0, max_y)
                else:
                    self.lr.move_to_angle(lr0, max_y)
                    self.lr.move_to_angle(lr0, min_y)

            # ---- 一圈結束 -------------------------------
            if not TEST_MODE:       # 現場只掃一圈
                break
            reverse = not reverse   # TEST_MODE 繼續下一圈

        self._close_actuators()

    # ---------- I/O 動作 ----------------------------------------------
    def _open_actuators(self):
        if TEST_MODE:
            for c in (COIL_LED_L, COIL_LED_R):
                self.modbus.write_coil(c, True, unit=UNIT)
        else:
            for c in (COIL_BALL, COIL_MOTOR):
                self.modbus.write_coil(c, True, unit=UNIT)

    def _close_actuators(self):
        if TEST_MODE:
            for c in (COIL_LED_L, COIL_LED_R):
                self.modbus.write_coil(c, False, unit=UNIT)
        else:
            for c in (COIL_BALL, COIL_MOTOR):
                self.modbus.write_coil(c, False, unit=UNIT)

    def _water_low(self):
        res = self.modbus.read_discrete_inputs(SENSOR_WATER, 1, unit=UNIT)
        return (not res.isError()) and (not res.bits[0])

    # ---------- 結束 ---------------------------------------------------
    def _cleanup(self):
        self._close_actuators()
        self.modbus.close(); self.up.close(); self.lr.close()
        self.get_logger().info(" Hardware closed, node idle")

# ---------------- main ----------------------------------------------
def main():
    rclpy.init()
    node = SprayExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(" Ctrl-C → shutdown")
    finally:
        node._cleanup()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
