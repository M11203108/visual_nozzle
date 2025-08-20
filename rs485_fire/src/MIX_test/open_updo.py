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
    packed = struct.pack('!f', f)
    return packed.hex()

# 讀取編碼器的原始值（未轉換為圈數）
def read_encoder_raw(client, unit):
    position_high = client.read_holding_registers(0x0007, 1, unit=unit).registers[0]
    position_low = client.read_holding_registers(0x0008, 1, unit=unit).registers[0]
    encoder_value = (position_high << 16) | position_low
    if encoder_value & 0x80000000:  # 處理負值
        encoder_value = -(0xFFFFFFFF - encoder_value + 1)
    return encoder_value

# 控制馬達移動至目標圈數
def move_motor(client, unit, direction, target_circle, set_rpm):
    ieee_circle = float_to_hex(target_circle)
    ieee_circle_high = int('0x' + ieee_circle[0:4], 16)
    ieee_circle_low = int('0x' + ieee_circle[4:8], 16)

    ieee_754_rpm = float_to_hex(set_rpm)
    ieee_754_rpm_high = int('0x' + ieee_754_rpm[0:4], 16)
    ieee_754_rpm_low = int('0x' + ieee_754_rpm[4:8], 16)

    client.write_register(0x004F, 0x0001, unit=unit)  # 啟動馬達
    client.write_register(0x0053, ieee_754_rpm_high, unit=unit)  # 設定轉速高位
    client.write_register(0x0054, ieee_754_rpm_low, unit=unit)  # 設定轉速低位
    client.write_register(0x0050, direction, unit=unit)  # 設置方向
    client.write_register(0x0055, ieee_circle_high, unit=unit)  # 設置目標圈數高位
    client.write_register(0x0056, ieee_circle_low, unit=unit)  # 設置目標圈數低位
    print(f"Motor moving: direction={direction}, target_circle={target_circle:.4f}")

# 根據編碼器值計算目標圈數並移動到指定位置
def move_to_zero_position(client, unit, current_encoder, zero_encoder, set_rpm, resolution):
    print(f"Moving to zero position: {zero_encoder} from {current_encoder}")

    # 計算目標圈數
    position_diff = abs(zero_encoder - current_encoder)
    target_circle = position_diff / resolution
    direction = 0x0000 if zero_encoder > current_encoder else 0x0001

    # 移動馬達
    move_motor(client, unit, direction, target_circle, set_rpm)

    # 等待移動完成
    while True:
        current_encoder = read_encoder_raw(client, unit)
        print(f"Current encoder: {current_encoder}, Target encoder: {zero_encoder}")
        if abs(current_encoder - zero_encoder) < 100:  # 設定允許的誤差範圍
            break
        time.sleep(0.1)

    # 停止馬達
    client.write_register(0x004F, 0x0000, unit=unit)
    print("Motor stopped at zero position.")

# 執行歸零校準
def zero_calibration(client, unit, resolution):
    print("Starting zero calibration...")

    # 找到上極限
    print("Moving to up limit...")
    up_limit = None
    stable_count = 0
    previous_encoder = None
    while True:
        move_motor(client, unit, 0x0001, 1.0, 200)  # 嘗試大範圍移動
        current_encoder = read_encoder_raw(client, unit)
        print(f"Current encoder: {current_encoder}")

        if previous_encoder == current_encoder:
            stable_count += 1
        else:
            stable_count = 0

        previous_encoder = current_encoder

        if stable_count >= 10:
            up_limit = current_encoder
            break

    print(f"Up limit: {up_limit}")

    # 找到下極限
    print("Moving to down limit...")
    down_limit = None
    stable_count = 0
    previous_encoder = None
    while True:
        move_motor(client, unit, 0x0000, 1.0, 200)  # 嘗試大範圍移動
        current_encoder = read_encoder_raw(client, unit)
        print(f"Current encoder: {current_encoder}")

        if previous_encoder == current_encoder:
            stable_count += 1
        else:
            stable_count = 0

        previous_encoder = current_encoder

        if stable_count >= 10:
            down_limit = current_encoder
            break

    print(f"Down limit: {down_limit}")

    # 設定下極限為 0 度基準
    zero_encoder = down_limit

    # 計算角度範圍
    down_limit_angle = 0.0
    up_limit_angle = (up_limit - down_limit) * 360.0 / resolution

    print(f"Zero position set to: {zero_encoder} (0 degrees)")
    print(f"Down limit: {down_limit}, Down limit angle: {down_limit_angle:.2f} degrees")
    print(f"Up limit: {up_limit}, Up limit angle: {up_limit_angle:.2f} degrees")

    # 移動到 0 度
    # move_to_zero_position(client, unit, read_encoder_raw(client, unit), zero_encoder, 50, resolution)

    return zero_encoder, down_limit, up_limit

# 主程式入口
def main():
    client = ModbusClient(
        method='rtu',
        port='/dev/updottyUSB',
        baudrate=9600,
        parity='N',
        stopbits=2,
        bytesize=8,
        timeout=3
    )

    UNIT = 0x00
    resolution = 3600 * 4  # 編碼器分辨率

    if not client.connect():
        print("Cannot connect to Modbus device.")
        return

    print("Connected")

    # 執行歸零校準
    zero_encoder, down_limit, up_limit = zero_calibration(client, UNIT, resolution)
    print(f"Calibration complete. Zero encoder: {zero_encoder}, Down limit: {down_limit}, Up limit: {up_limit}")
    print(f"Down limit angle: 0.00 degrees")
    print(f"Up limit angle: {(up_limit - down_limit) * 360.0 / resolution:.2f} degrees")
    
    # 完成校準後
    calibration_result = {
        "zero_encoder": zero_encoder,
        "down_limit": down_limit,
        "up_limit": up_limit,
        "down_limit_angle": 0.0,
        "up_limit_angle": (up_limit - down_limit) * 360.0 / resolution
    }

    # 存入相對路徑
    save_calibration_result("calibration_updo.json", calibration_result)

    client.close()
    print("Connection closed")

if __name__ == "__main__":
    main()
