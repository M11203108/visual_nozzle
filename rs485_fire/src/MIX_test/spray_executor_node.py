#!/usr/bin/env python3
# auto_spray_executor.py - 自動掃描啟動版本，沿用手動版本邏輯，但移除手動 input

from motor_control.motor1 import MotorControllerupdo  # 控制上下馬達
from motor_control.motor2 import MotorControllerLR     # 控制左右馬達
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


import json
import csv
import os
import time
import sys

# Modbus 設定
UNIT = 0x01
sensor_water = 0x0004
# coil_ball = 0x0006
# coil_motor = 0x0003
# coil_ledL = 0x0004             # 左側 LED
# coil_ledR = 0x0005             # 右側 LED

#test
coil_ball = 0x0006
coil_motor = 0x0003

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
def load_trajectory_from_csv():
    # base_path = os.path.dirname(os.path.abspath(__file__))
    file_path = "/home/robot/newjasmine_ws/src/rs485_fire/config/trajectory_angles.csv"
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

# === 水位感測器讀取 ===
def read_water_sensor(client):
    result = client.read_discrete_inputs(sensor_water, count=1, unit=UNIT)
    if not result.isError():
        return not result.bits[0]  # False 表示觸發
    return False

# === 關閉元件 ===
def stop_valve_and_pump(client):
    client.write_coil(coil_ball, False, unit=UNIT)
    client.write_coil(coil_motor, False, unit=UNIT)
    print("🚫 球閥與泵浦已關閉")

# === 主程式（自動版） ===
def main():
    rclpy.init()
    node = rclpy.create_node("spray_executor_status_node")
    status_pub = node.create_publisher(String, "/spray_status", 10)

    # 發布開始狀態
    status_pub.publish(String(data="start"))
    print("📡 發布 /spray_status = start")


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

    trajectory_list = load_trajectory_from_csv()
    if not trajectory_list:
        print("[ERROR] No trajectory data found. Exiting.")
        return

    segments = segment_trajectory(trajectory_list)
    up_zero = up_calib["zero_encoder"]
    lr_zero = lr_calib["zero_encoder"]
    up_min_angle, up_max_angle = up_calib["down_limit_angle"], up_calib["up_limit_angle"]
    lr_min_angle, lr_max_angle = lr_calib["right_limit_angle"], lr_calib["left_limit_angle"]

    # 開啟球閥與馬達
    client.write_coil(coil_ball, True, unit=UNIT)
    client.write_coil(coil_motor, True, unit=UNIT)
    print("🔥 開始自動掃描")
    
    reverse = False
    start_time = time.time()  # 加入這行：紀錄開始時間
    try:
        while True:
            # 加入這段：檢查是否超過 40 秒
            elapsed = time.time() - start_time
            if elapsed > 40:
                print("⏱️ 20秒已到，自動中止掃描")
                break

            if read_water_sensor(client):
                print("💧 水位過低，中止掃描")
                break

            segment_iter = segments if not reverse else reversed(segments)

            for idx, (pitch, yaws) in enumerate(segment_iter):
                if read_water_sensor(client):
                    print("💧 水位中止段")
                    break

                hardware_pitch = 26.0 + pitch
                print(f"\n[Segment {idx+1}] Pitch: {pitch:.2f}° → {hardware_pitch:.2f}° ({len(yaws)} yaws)")

                if not (up_min_angle <= hardware_pitch <= up_max_angle):
                    print(f"[ERROR] Up-Down angle {hardware_pitch:.2f} out of range. Skipping.")
                    continue

                move_motor_thread(up_motor, up_zero, hardware_pitch, "Up-Down Motor")

                min_yaw = min(yaws)
                max_yaw = max(yaws)
                if not (lr_min_angle <= min_yaw <= lr_max_angle) or not (lr_min_angle <= max_yaw <= lr_max_angle):
                    print("[ERROR] Yaw angle out of range. Skipping.")
                    continue

                if not reverse:
                    move_motor_thread(lr_motor, lr_zero, min_yaw, "Left-Right Motor")
                    move_motor_thread(lr_motor, lr_zero, max_yaw, "Left-Right Motor")
                else:
                    move_motor_thread(lr_motor, lr_zero, max_yaw, "Left-Right Motor")
                    move_motor_thread(lr_motor, lr_zero, min_yaw, "Left-Right Motor")

            reverse = not reverse


    except KeyboardInterrupt:
        print("[EXIT] 使用者手動中止")

    finally:
        stop_valve_and_pump(client)
        up_motor.close()
        lr_motor.close()
        client.close()
        print("✅ 掃描完成並關閉所有連線")
        # 發布結束狀態 5 次（每秒一次）
        end_time = time.time() + 5
        while time.time() < end_time and rclpy.ok():
            status_pub.publish(String(data="done"))
            time.sleep(1.0)
        print("📡 發布 /spray_status = done")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
