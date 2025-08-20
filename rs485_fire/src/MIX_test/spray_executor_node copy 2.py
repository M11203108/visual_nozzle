#!/usr/bin/env python3
# auto_spray_executor.py ------------------------------------------------
#  • 先等待 TRIGGER_TOPIC (default /plan_status == 'finished')
#  • TEST_MODE = False → 水位感測器決定停止
#  • TEST_MODE = True  → 30 s 後強制停止 & 5 次 /spray_status=done
# ----------------------------------------------------------------------

TEST_MODE      = True                 # ← False = 現場  True = 測試
TRIGGER_TOPIC  = "/plan_status"       # ← 觸發掃描用 topic
TRIGGER_MSG    = "finished"           # ← 收到這個字串才開工

from motor_control.motor1 import MotorControllerupdo
from motor_control.motor2 import MotorControllerLR
from pymodbus.client.sync  import ModbusSerialClient as ModbusClient
import json, csv, os, time, threading, sys

# ── rclpy 僅在需要時才 import ────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos    import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
# -----------------------------------------------------------------------
UNIT          = 0x01
sensor_water  = 0x0004
coil_ledL     = 0x0004
coil_ledR     = 0x0005
coil_ball     = 0x0006             # 球閥
coil_motor    = 0x0003            # 泵浦

BASE = os.path.dirname(os.path.abspath(__file__))

# ---------- 舊工具函式 --------------------------------------------------
def load_calib(f):
    with open(os.path.join(BASE, f)) as fp: return json.load(fp)

def load_traj():
    p = os.path.join(BASE, "trajectories/trajectory_angles.csv")
    with open(p) as fp:
        return [(float(r['pitch_deg']), float(r['yaw_deg']))
                for r in csv.DictReader(fp)]

def segment(traj):
    out, cur, buf = [], None, []
    for p,y in traj:
        if p!=cur:
            if buf: out.append((cur,buf))
            cur, buf = p,[]
        buf.append(y)
    if buf: out.append((cur,buf))
    return out

def move(motor, zero, tgt, name):
    try:
        print(f"{name} → {tgt:.2f}°")
        motor.move_to_angle(zero, tgt)
        time.sleep(0.05)
    except Exception as e:
        print(f"[ERR] {name}: {e}")
# -----------------------------------------------------------------------

def wait_for_trigger():
    """阻塞直到收到指定 topic / 字串"""
    print(f"⏳ Waiting {TRIGGER_TOPIC} == '{TRIGGER_MSG}' …")

    rclpy.init()
    evt  = threading.Event()

    class _TmpNode(Node):          # ← 直接繼承 rclpy.node.Node
        pass
    node = _TmpNode("spray_waiter")

    def _cb(msg):
        if msg.data.strip() == TRIGGER_MSG:
            evt.set()
    node.create_subscription(String, TRIGGER_TOPIC, _cb, 10)

    while rclpy.ok() and not evt.is_set():
        rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()
    print("🚀 Trigger received – start scanning")

def publish_done():
    """TEST_MODE 結束時 1 Hz × 5 次送 /spray_status=done"""
    qos = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability =QoSDurabilityPolicy.TRANSIENT_LOCAL)
    rclpy.init()
    node = rclpy.create_node("spray_status_pub")
    pub  = node.create_publisher(String, "/spray_status", qos)
    for _ in range(5):
        pub.publish(String(data="done"))
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    print("📣 /spray_status=done sent (5×)")

# ---------- 主程式 -----------------------------------------------------
def main():
    # #test###########

    # wait_for_trigger()  # ← 仍保留觸發機制

    # if TEST_MODE:
    #     print("🧪 TEST_MODE: skip all hardware, wait 5 sec then publish done")
    #     time.sleep(5)
    #     publish_done()
    #     return  # ⬅️ 這樣就不會跑後面連線、馬達控制流程了
    #  #test###########


    wait_for_trigger()                       # ← 先等觸發 topic

    # ── Modbus & Motor 連線 ──────────────────────────────────
    mb = ModbusClient(method='rtu', port='/dev/IOttyUSB',
                      baudrate=9600, stopbits=2, bytesize=8, timeout=3)
    if not mb.connect():
        print("[ERR] Modbus connect fail"); sys.exit(1)

    up = MotorControllerupdo(port='/dev/updottyUSB', baudrate=9600, unit=0x00)
    lr = MotorControllerLR(port='/dev/LRttyUSB', baudrate=9600, unit=0x01)
    if not (up.connect() and lr.connect()):
        print("[ERR] Motor connect fail"); sys.exit(1)

    up_c = load_calib("calibration_updo.json")
    lr_c = load_calib("calibration_LR.json")
    segs = segment(load_traj())

    up0, up_lo, up_hi = up_c["zero_encoder"], up_c["down_limit_angle"], up_c["up_limit_angle"]
    lr0, lr_lo, lr_hi = lr_c["zero_encoder"], lr_c["right_limit_angle"], lr_c["left_limit_angle"]

    for c in (coil_ball, coil_motor): mb.write_coil(c, True, unit=UNIT)
    print("🔥 Scan started")

    t0, reverse = time.time(), False
    try:
        while True:
            if TEST_MODE and time.time() - t0 >= 30:
                print(" 30 s timeout (TEST_MODE)")
                break

            if not TEST_MODE:
                res = mb.read_discrete_inputs(sensor_water, 1, unit=UNIT)
                if (not res.isError()) and (not res.bits[0]):
                    print("💧 Water LOW – stop")
                    break

            for pitch, yaws in (segs if not reverse else reversed(segs)):
                if TEST_MODE and time.time() - t0 >= 30:
                    break

                # ─────────── Up-Down ────────────
                hw_p = 26.0 + pitch
                if not up_lo <= hw_p <= up_hi:
                    continue
                move(up, up0, hw_p, "Up-Down")

                # ─────────── Left-Right ─────────
                y1, y2 = min(yaws), max(yaws)
                if not (lr_lo <= y1 <= lr_hi and lr_lo <= y2 <= lr_hi):
                    continue

                # 先決定順序
                sweep = (y1, y2) if not reverse else (y2, y1)

                for tgt in sweep:
                    # ① 讀取目前角度（簡易版：用 motor.read_encoder_raw→角度）
                    try:
                        enc = lr.read_encoder_raw()
                        cur_ang = (enc - lr0) * 360.0 / lr.resolution
                    except Exception:
                        cur_ang = None     # 讀失敗就直接送指令

                    # ② 若差值 < 1° 就跳過
                    if cur_ang is not None and abs(cur_ang - tgt) < 0.1:
                        continue

                    move(lr, lr0, tgt, "Left-Right")
                    # time.sleep(3)      # ③ 300 ms 靜默，防衝突

            reverse = not reverse

    finally:
        for c in (coil_ball, coil_motor): mb.write_coil(c, False, unit=UNIT)
        up.close(); lr.close(); mb.close()
        print("✅ Hardware closed")

        if TEST_MODE:
            publish_done()

        if rclpy.ok():      # ✅ 多這行保險結尾
            rclpy.shutdown()

if __name__ == "__main__":
    main()
