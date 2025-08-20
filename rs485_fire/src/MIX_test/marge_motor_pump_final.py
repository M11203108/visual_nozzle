#!/usr/bin/env python3
# spray_executor.py  (TEST-ONLY: motors & Modbus are mocked by print)

import os, csv, json, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ───── 測試旗標 ───────────────────────────────────────────────
USE_HARDWARE = False   # ← 只要改成 True 就會重新啟用真馬達/Modbus

if USE_HARDWARE:
    from motor_control.motor1 import MotorControllerupdo
    from motor_control.motor2 import MotorControllerLR
    from pymodbus.client.sync import ModbusSerialClient as ModbusClient
    UNIT, sensor_water = 0x01, 0x0004
    coil_ledL, coil_ledR = 0x0004, 0x0005

# ───── 讀檔工具 ───────────────────────────────────────────────
def load_json(fname):
    path = os.path.join(os.path.dirname(__file__), fname)
    with open(path) as f:
        return json.load(f)

def load_traj(csv_name="trajectories/trajectory_angles.csv"):
    path = os.path.join(os.path.dirname(__file__), csv_name)
    with open(path) as f:
        return [(float(r['pitch_deg']), float(r['yaw_deg']))
                for r in csv.DictReader(f)]

def segment(traj):
    segs, cur, buf = [], None, []
    for p, y in traj:
        if p != cur:
            if buf: segs.append((cur, buf))
            buf, cur = [], p
        buf.append(y)
    if buf: segs.append((cur, buf))
    return segs

# ───── Node ──────────────────────────────────────────────────
class SprayExecutor(Node):
    def __init__(self):
        super().__init__("spray_executor")
        self.create_subscription(String, "/plan_status", self._status_cb, 10)
        self.pub = self.create_publisher(String, "/spray_status", 10)
        self.get_logger().info("🟢 等待 /plan_status == 'finished'…")

        if USE_HARDWARE:
            # 連接硬體
            self._init_hw()
        else:
            self.get_logger().warn("⚠️ TEST MODE：所有硬體操作以 print 取代")

    # ---------- topic callback --------------------------------
    def _status_cb(self, msg: String):
        print(f"[plan_status] {msg.data}")                    # ← 觀察所有訊息
        if msg.data != "finished":
            return
        self.get_logger().info("🔔 收到 finished，開始『模擬』噴灑")

        try:
            self._run_scan_mock()
            self.pub.publish(String(data="done"))
            self.get_logger().info("✅ 噴灑結束 /spray_status=done")
        except Exception as e:
            self.get_logger().error(f"⚠️ 發生例外：{e}")
            self.pub.publish(String(data="aborted"))

    # ---------- 測試版掃描 ------------------------------------
    def _run_scan_mock(self):
        traj = load_traj()
        segs = segment(traj)

        print(f"[Trajectory] {len(traj)} points / {len(segs)} segments")
        for idx, (pitch, yaws) in enumerate(segs):
            hw_pitch = 26.0 + pitch
            print(f"  ▸ Segment {idx+1}: pitch={pitch:.1f}° "
                  f"(hw {hw_pitch:.1f}°) yaw {min(yaws):.1f}~{max(yaws):.1f}")
            time.sleep(0.1)   # 模擬動作延時

        print("💧 LED ON  → 模擬噴灑…")
        time.sleep(1.0)
        print("💧 LED OFF → 噴灑完成")

    # ---------- 真硬體初始化 (只在 USE_HARDWARE=True 時用) -----
    def _init_hw(self):
        self.get_logger().info("🛠️  連接 Modbus / Motors")
        self.modbus = ModbusClient(method='rtu', port='/dev/IOttyUSB',
                                   baudrate=9600, stopbits=2,
                                   bytesize=8, timeout=3)
        if not self.modbus.connect():
            self.get_logger().error("Modbus 連線失敗"); sys.exit(1)

        self.up_cfg = load_json("calibration_updo.json")
        self.lr_cfg = load_json("calibration_LR.json")
        self.up = MotorControllerupdo('/dev/updottyUSB', 9600, 0x00)
        self.lr = MotorControllerLR('/dev/LRttyUSB', 9600, 0x01)
        if not (self.up.connect() and self.lr.connect()):
            self.get_logger().error("Motor 連線失敗"); sys.exit(1)

    def cleanup(self):
        if USE_HARDWARE:
            self.modbus.write_coil(coil_ledL, False, unit=UNIT)
            self.modbus.write_coil(coil_ledR, False, unit=UNIT)
            self.up.close(); self.lr.close(); self.modbus.close()
        self.get_logger().info("🔚 node 結束")

# ───── main ─────────────────────────────────────────────────
def main():
    rclpy.init()
    node = SprayExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
