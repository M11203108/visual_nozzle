from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time
import struct
import json
import os

# 獲取當前檔案的資料夾
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

def save_calibration_result(filename, result):
    """
    保存校準結果為 JSON，確保存放在相對位置。
    """
    file_path = os.path.join(BASE_DIR, filename)
    with open(file_path, 'w') as f:
        json.dump(result, f)
    print(f"Calibration results saved to {file_path}")

# 將浮點數轉換為16進位表示的字串
def float_to_hex(f):
    """
    將浮點數轉換為 IEEE 754 標準格式的 16 進位字串表示，用於通訊協議中的數據傳輸。
    """
    packed = struct.pack('!f', f)  # 使用 big-endian 格式打包浮點數
    return packed.hex()  # 轉換為 16 進位字串

# 讀取編碼器的原始值（未轉換為圈數）
def read_encoder_raw(client, unit):
    """
    從編碼器讀取原始數值，並進行符號處理以處理可能的負值。
    """
    # 從兩個寄存器讀取高位和低位數值
    position_high = client.read_holding_registers(0x0007, 1, unit=unit).registers[0]
    position_low = client.read_holding_registers(0x0008, 1, unit=unit).registers[0]
    encoder_value = (position_high << 16) | position_low  # 合併高低位數據

    # 檢查負值，並進行二補數轉換
    if encoder_value & 0x80000000:
        encoder_value = -(0xFFFFFFFF - encoder_value + 1)
    return encoder_value

# 控制馬達移動至目標圈數
def move_motor(client, unit, direction, target_circle, set_rpm):
    """
    發送命令至馬達，控制其移動至指定目標圈數，並設置方向及速度。
    """
    # 將目標圈數和速度轉換為 IEEE 754 格式
    ieee_circle = float_to_hex(target_circle)
    ieee_circle_high = int('0x' + ieee_circle[0:4], 16)
    ieee_circle_low = int('0x' + ieee_circle[4:8], 16)

    ieee_754_rpm = float_to_hex(set_rpm)
    ieee_754_rpm_high = int('0x' + ieee_754_rpm[0:4], 16)
    ieee_754_rpm_low = int('0x' + ieee_754_rpm[4:8], 16)

    # 發送控制命令到相關寄存器
    client.write_register(0x004F, 0x0001, unit=unit)  # 啟動馬達
    client.write_register(0x0053, ieee_754_rpm_high, unit=unit)  # 設定轉速高位
    client.write_register(0x0054, ieee_754_rpm_low, unit=unit)  # 設定轉速低位
    client.write_register(0x0050, direction, unit=unit)  # 設置方向
    client.write_register(0x0055, ieee_circle_high, unit=unit)  # 設置目標圈數高位
    client.write_register(0x0056, ieee_circle_low, unit=unit)  # 設置目標圈數低位
    print(f"[DEBUG] Motor command sent -> Direction: {direction}, Target circle: {target_circle:.4f}, RPM: {set_rpm}")

# 根據與目標的角度差距計算速度
def calculate_rpm_by_distance(angle_diff):
    """
    根據角度差距動態計算適合的 RPM（轉速）。
    """
    abs_diff = abs(angle_diff)  # 取絕對值
    if abs_diff > 120:
        return 50  # 大角度使用最高速
    elif 120 >= abs_diff > 70:
        return 40
    elif 70 >= abs_diff > 60:
        return 30
    elif 60 >= abs_diff > 40:
        return 20
    elif 40 >= abs_diff > 5:
        return 10
    else:
        return 10  # 小範圍微調速度

# 移動馬達至目標位置
def move_to_target_position(client, unit, current_encoder, target_encoder, resolution):
    """
    控制馬達從當前位置移動到目標位置，並依據角度差距動態調整速度。
    """
    print(f"Moving to target position: {target_encoder} from {current_encoder}")

    while True:
        current_encoder = read_encoder_raw(client, unit)  # 獲取當前編碼器值
        angle_diff = (target_encoder - current_encoder) * 360.0 / resolution  # 計算角度差距
        current_angle = current_encoder * 360.0 / resolution  # 當前角度
        target_angle = target_encoder * 360.0 / resolution  # 目標角度

        # 停止條件：當角度差小於 1 度時停止
        if abs(angle_diff) < 1.0:
            client.write_register(0x004F, 0x0000, unit=unit)  # 停止馬達
            print(f"Reached target position: {current_encoder}")
            break

        # 動態計算轉速
        set_rpm = calculate_rpm_by_distance(angle_diff)

        # 設定移動方向
        direction = 0x0000 if angle_diff > 0 else 0x0001
        target_circle = abs(angle_diff / 360.0)  # 計算目標圈數

        # 發送移動命令
        move_motor(client, unit, direction, target_circle, set_rpm)

# 執行歸零校準
def zero_calibration(client, unit, resolution):
    """
    進行歸零校準，尋找左極限與右極限，並計算零點位置。
    """
    print("Starting zero calibration...")

    # 左極限
    print("Moving to left limit...")
    left_limit = None
    stable_count = 0
    previous_encoder = None
    while True:
        move_motor(client, unit, 0x0000, 1.0, 50)  # 左移
        current_encoder = read_encoder_raw(client, unit)
        if previous_encoder == current_encoder:
            stable_count += 1  # 穩定計數
        else:
            stable_count = 0
        previous_encoder = current_encoder
        if stable_count >= 10:
            left_limit = current_encoder
            break
    print(f"Left limit: {left_limit}")

    # 右極限
    print("Moving to right limit...")
    right_limit = None
    stable_count = 0
    previous_encoder = None
    while True:
        move_motor(client, unit, 0x0001, 1.0, 50)  # 右移
        current_encoder = read_encoder_raw(client, unit)
        if previous_encoder == current_encoder:
            stable_count += 1
        else:
            stable_count = 0
        previous_encoder = current_encoder
        if stable_count >= 10:
            right_limit = current_encoder
            break
    print(f"Right limit: {right_limit}")

    # 設置零點
    zero_encoder = (left_limit + right_limit) // 2
    print(f"Zero position set to: {zero_encoder}")

    # 移動到零點
    move_to_target_position(client, unit, read_encoder_raw(client, unit), zero_encoder, resolution)
    return zero_encoder, left_limit, right_limit

# 主程式入口
def main():
    """
    主程式，負責初始化 Modbus 客戶端，執行校準與角度控制邏輯。
    """
    client = ModbusClient(
        method='rtu',
        port='/dev/LRttyUSB',
        baudrate=9600,
        parity='N',
        stopbits=2,
        bytesize=8,
        timeout=3
    )

    UNIT = 0x01  # Modbus 單元 ID
    resolution = 3600 * 4  # 編碼器分辨率

    if not client.connect():
        print("Cannot connect to Modbus device.")
        return

    print("Connected")

    # 執行歸零校準
    zero_encoder, left_limit, right_limit = zero_calibration(client, UNIT, resolution)
    print(f"Calibration complete. Zero encoder: {zero_encoder}, Left limit: {left_limit}, Right limit: {right_limit}")

    # 完成校準後
    calibration_result = {
        "zero_encoder": zero_encoder,
        "left_limit": left_limit,
        "right_limit": right_limit,
        "left_limit_angle": (left_limit - zero_encoder) * 360.0 / resolution,
        "right_limit_angle": (right_limit - zero_encoder) * 360.0 / resolution
    }

    # 存入相對路徑
    save_calibration_result("calibration_LR.json", calibration_result)

    client.close()
    print("Connection closed")

if __name__ == "__main__":
    main()
