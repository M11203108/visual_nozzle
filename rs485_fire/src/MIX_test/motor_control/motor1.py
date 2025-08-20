from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time
import struct
import json
import os

# 取得目前檔案的資料夾位置
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 取得 `MIX_test` 上層資料夾
PARENT_DIR = os.path.dirname(BASE_DIR)

def load_calibration_result(filename):
    """
    從上層 `MIX_test/` 資料夾讀取校準數據
    """
    file_path = os.path.join(PARENT_DIR, filename)
    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"[ERROR] Calibration file {file_path} not found.")
        return None

updo_result = load_calibration_result("calibration_updo.json")

if updo_result:
    print("Loaded calibration result for updo:")
    print(updo_result)

class MotorControllerupdo:
    def __init__(self, port, unit, resolution=3600 * 4, baudrate=9600):
        """
        初始化馬達控制器。
        :param port: Modbus 端口（如 /dev/updottyUSB）
        :param unit: 馬達 Modbus ID
        :param resolution: 編碼器分辨率，默認 3600 * 4
        :param baudrate: 通信波特率，默認 9600
        """
        self.client = ModbusClient(
            method='rtu',
            port=port,
            baudrate=baudrate,
            parity='N',
            stopbits=2,
            bytesize=8,
            timeout=3
        )
        self.unit = unit
        self.resolution = resolution

    def connect(self):
        """連接 Modbus 馬達"""
        return self.client.connect()

    def close(self):
        """關閉 Modbus 連接"""
        self.client.close()

    @staticmethod
    def float_to_hex(f):
        """將浮點數轉換為 IEEE 754 標準格式的 16 進位字串表示"""
        packed = struct.pack('!f', f)
        return packed.hex()

    def read_encoder_raw(self):
        """讀取編碼器的原始值"""
        try:
            response_high = self.client.read_holding_registers(0x0007, 1, unit=self.unit)
            response_low = self.client.read_holding_registers(0x0008, 1, unit=self.unit)
            position_high = response_high.registers[0]
            position_low = response_low.registers[0]
            encoder_value = (position_high << 16) | position_low
            if encoder_value & 0x80000000:  # 處理負值
                encoder_value = -(0xFFFFFFFF - encoder_value + 1)
            return encoder_value
        except Exception as e:
            print(f"[ERROR] Failed to read encoder raw value: {e}")
            return 0

    def move_motor(self, direction, target_circle, set_rpm):
        """控制馬達移動到目標圈數"""
        try:
            ieee_circle = self.float_to_hex(target_circle)
            ieee_circle_high = int('0x' + ieee_circle[0:4], 16)
            ieee_circle_low = int('0x' + ieee_circle[4:8], 16)

            ieee_rpm = self.float_to_hex(set_rpm)
            ieee_rpm_high = int('0x' + ieee_rpm[0:4], 16)
            ieee_rpm_low = int('0x' + ieee_rpm[4:8], 16)

            self.client.write_register(0x004F, 0x0001, unit=self.unit)  # 啟動馬達
            self.client.write_register(0x0053, ieee_rpm_high, unit=self.unit)  # 設定轉速高位
            self.client.write_register(0x0054, ieee_rpm_low, unit=self.unit)  # 設定轉速低位
            self.client.write_register(0x0050, direction, unit=self.unit)  # 設置方向
            self.client.write_register(0x0055, ieee_circle_high, unit=self.unit)  # 設置目標圈數高位
            self.client.write_register(0x0056, ieee_circle_low, unit=self.unit)  # 設置目標圈數低位
            print(f"Motor {self.unit} moving: direction={direction}, target_circle={target_circle:.4f}")
        except Exception as e:
            print(f"[ERROR] Failed to move motor: {e}")

    def zero_calibration(self):
        """執行歸零校準，返回零點、上下極限"""
        print("Starting zero calibration...")
        up_limit, down_limit = None, None
        stable_count, previous_encoder = 0, None

        # 找到上極限
        print("Moving to up limit...")
        while True:
            self.move_motor(0x0001, 1.0, 300)  # 嘗試大範圍移動
            current_encoder = self.read_encoder_raw()
            print(f"Current encoder (up): {current_encoder}")

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
        stable_count, previous_encoder = 0, None
        while True:
            self.move_motor(0x0000, 1.0, 300)  # 嘗試大範圍移動
            current_encoder = self.read_encoder_raw()
            print(f"Current encoder (down): {current_encoder}")

            if previous_encoder == current_encoder:
                stable_count += 1
            else:
                stable_count = 0

            previous_encoder = current_encoder

            if stable_count >= 10:
                down_limit = current_encoder
                break
        print(f"Down limit: {down_limit}")

        zero_encoder = down_limit
        print(f"Zero position set to: {zero_encoder}")
        return zero_encoder, down_limit, up_limit

    def move_to_angle(self, zero_encoder, target_angle):
        """移動到指定角度"""
        try:
            current_encoder = self.read_encoder_raw()
            current_angle = (current_encoder - zero_encoder) * 360.0 / self.resolution
            angle_diff = target_angle - current_angle

            if abs(angle_diff) < 1.0:
                print(f"Motor {self.unit} already at target angle: {target_angle}")
                return

            direction = 0x0001 if angle_diff > 0 else 0x0000
            target_circle = abs(angle_diff / 360.0)
            self.move_motor(direction, target_circle, 300)

            # 等待運動完成
            while True:
                current_encoder = self.read_encoder_raw()
                current_angle = (current_encoder - zero_encoder) * 360.0 / self.resolution
                if abs(current_angle - target_angle) < 1.0:
                    self.client.write_register(0x004F, 0x0000, unit=self.unit)  # 停止馬達
                    print(f"Reached target angle: {target_angle:.2f}")
                    break
                
                time.sleep(0.05)  # 等待 25 毫秒以避免過於頻繁的讀取
        except Exception as e:
            print(f"[ERROR] Failed to move to target angle: {e}")


# 測試程式入口
if __name__ == "__main__":
    motor = MotorControllerupdo(port='/dev/updottyUSB', unit=0x00)

    if not motor.connect():
        print("Cannot connect to motor.")
    else:
        print("Motor connected.")
        try:
            # 載入校準結果
            calibration_result = load_calibration_result("calibration_updo.json")

            if calibration_result:
                zero_encoder = calibration_result["zero_encoder"]
                down_limit = calibration_result["down_limit"]
                up_limit = calibration_result["up_limit"]

                print(f"Calibration loaded: Zero={zero_encoder}, Down={down_limit}, Up={up_limit}")
                # 移動到指定角度，例如 30 度
                motor.move_to_angle(zero_encoder, 0)
            else:
                print("[ERROR] Calibration data not available.")
        except Exception as e:
            print(f"[ERROR] {e}")
        finally:
            motor.close()
            print("Connection closed.")
