from motor_control.motor1 import MotorControllerupdo  # 控制上下馬達
from motor_control.motor2 import MotorControllerLR   # 控制左右馬達
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException

import json
import csv
import os
import time

# Modbus 線圈與感測器地址定義
UNIT = 0x01
coil_button = 0x0001           # 啟動按鈕
sensor_water = 0x0004          # 水位感測器
coil_ball = 0x0006             # 球閥
coil_motor = 0x0003            # 泵浦
coil_ledL = 0x0004             # 左側 LED
coil_ledR = 0x0005             # 右側 LED

# === 讀取校準檔 ===
def load_calibration(filename):
    base_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(base_path, filename)
    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"[ERROR] Calibration file {file_path} not found.")
        return None

# === 載入 CSV 軌跡檔 ===
def load_trajectory_from_csv(filename="trajectory_angles.csv"):
    base_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(base_path, filename)
    trajectory = []
    try:
        with open(file_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                pitch = float(row['pitch_deg'])
                yaw = float(row['yaw_deg'])
                trajectory.append((pitch, yaw))
        print(f"[INFO] Loaded {len(trajectory)} points from {file_path}")
    except FileNotFoundError:
        print(f"[ERROR] Trajectory file {file_path} not found.")
    return trajectory

# === 分段 ===
def segment_trajectory(trajectory_list):
    segments = []
    current_pitch = None
    current_segment = []

    for pitch, yaw in trajectory_list:
        if pitch != current_pitch:
            if current_segment:
                segments.append((current_pitch, current_segment))
            current_segment = []
            current_pitch = pitch
        current_segment.append(yaw)

    if current_segment:
        segments.append((current_pitch, current_segment))

    return segments

# === 馬達移動 ===
def move_motor_thread(motor, zero_encoder, target_angle, name):
    try:
        print(f"{name} moving to {target_angle:.2f} degrees...")
        motor.move_to_angle(zero_encoder, target_angle)
        print(f"{name} reached {target_angle:.2f} degrees.")
    except Exception as e:
        print(f"[ERROR] {name} failed to move: {e}")

# === Modbus 控制函數 ===
def read_button(client):
    result = client.read_discrete_inputs(coil_button, count=1, unit=UNIT)
    if not result.isError():
        return result.bits[0]
    return False

def read_water_sensor(client):
    result = client.read_discrete_inputs(sensor_water, count=1, unit=UNIT)
    if not result.isError():
        return not result.bits[0]  # False 表示觸發
    return False

def stop_valve_and_pump(client):
    client.write_coil(coil_ball, False, unit=UNIT)
    client.write_coil(coil_motor, False, unit=UNIT)
    # client.write_coil(coil_ledL, False, unit=UNIT)
    # client.write_coil(coil_ledR, False, unit=UNIT)
    print("🚫 球閥、泵浦與 LED 已關閉")

# === 主程式 ===
def main():
    client = ModbusClient(method='rtu', port='/dev/IOttyUSB', baudrate=9600,
                          stopbits=2, bytesize=8, timeout=3)
    if not client.connect():
        print("[ERROR] Cannot connect to Modbus device.")
        return

    up_calib = load_calibration("calibration_updo.json")
    lr_calib = load_calibration("calibration_LR.json")
    if not up_calib or not lr_calib:
        print("[ERROR] Calibration data not available. Exiting program.")
        return

    up_motor = MotorControllerupdo(port='/dev/updottyUSB', baudrate=9600, unit=0x00)
    lr_motor = MotorControllerLR(port='/dev/LRttyUSB', baudrate=9600, unit=0x01)
    if not up_motor.connect():
        print("Cannot connect to Up-Down Motor.")
        return
    if not lr_motor.connect():
        print("Cannot connect to Left-Right Motor.")
        return

    trajectory_list = load_trajectory_from_csv("trajectories/trajectory_angles.csv")
    if not trajectory_list:
        print("[ERROR] No trajectory data found. Exiting.")
        return

    segments = segment_trajectory(trajectory_list)
    up_zero = up_calib["zero_encoder"]
    lr_zero = lr_calib["zero_encoder"]
    up_min_angle, up_max_angle = up_calib["down_limit_angle"], up_calib["up_limit_angle"]
    lr_min_angle, lr_max_angle = lr_calib["right_limit_angle"], lr_calib["left_limit_angle"]

    print("\n✅ 系統待命中。輸入 'o' 開始，'c' 結束，或水位感測器停止。\n")

    try:
        while True:
            user_input = input("請輸入指令 (o 啟動 / c 結束): ").strip().lower()
            if user_input == 'o':
                print("🔥 啟動掃描")
                reverse = False

                # 同步啟動泵浦與 LED
                client.write_coil(coil_ball, True, unit=UNIT)
                client.write_coil(coil_motor, True, unit=UNIT)
                # client.write_coil(coil_ledL, True, unit=UNIT)
                # client.write_coil(coil_ledR, True, unit=UNIT)

                while True:
                    if read_water_sensor(client):
                        print("💧 水位感測器觸發，中止掃描")
                        stop_valve_and_pump(client)
                        break
                    if user_input == 'c':
                        print("🛑 收到結束指令，中止掃描")
                        stop_valve_and_pump(client)
                        break

                    segment_iter = segments if not reverse else reversed(segments)

                    for idx, (pitch, yaws) in enumerate(segment_iter):
                        if read_water_sensor(client):
                            print("💧 感測器中止段")
                            stop_valve_and_pump(client)
                            break

                        hardware_pitch = 26.0 + pitch
                        print(f"\n[Segment {idx+1}] Pitch: {pitch:.2f}° (hardware: {hardware_pitch:.2f}°) ({len(yaws)} Yaw Points)")

                        if not (up_min_angle <= hardware_pitch <= up_max_angle):
                            print(f"[ERROR] Up-Down angle {hardware_pitch:.2f} out of range. Skipping.")
                            continue

                        move_motor_thread(up_motor, up_zero, hardware_pitch, "Up-Down Motor")

                        min_yaw = min(yaws)
                        max_yaw = max(yaws)

                        if not (lr_min_angle <= min_yaw <= lr_max_angle) or not (lr_min_angle <= max_yaw <= lr_max_angle):
                            print(f"[ERROR] Yaw angle out of range. Skipping.")
                            continue

                        if not reverse:
                            print(f"Scanning Yaw from {min_yaw:.2f}° to {max_yaw:.2f}°")
                            move_motor_thread(lr_motor, lr_zero, min_yaw, "Left-Right Motor")
                            move_motor_thread(lr_motor, lr_zero, max_yaw, "Left-Right Motor")
                        else:
                            print(f"Scanning Yaw from {max_yaw:.2f}° back to {min_yaw:.2f}°")
                            move_motor_thread(lr_motor, lr_zero, max_yaw, "Left-Right Motor")
                            move_motor_thread(lr_motor, lr_zero, min_yaw, "Left-Right Motor")

                    reverse = not reverse
            elif user_input == 'c':
                print("🛑 手動結束指令收到，停止所有動作")
                stop_valve_and_pump(client)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[EXIT] 使用者手動中止程序")
    finally:
        stop_valve_and_pump(client)
        up_motor.close()
        lr_motor.close()
        client.close()
        print("Connections closed.")

if __name__ == "__main__":
    main()
