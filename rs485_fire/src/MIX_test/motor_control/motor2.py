import time
import struct
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
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
        
LR_result = load_calibration_result("calibration_LR.json")
if LR_result:
    print("Loaded calibration result for LR:")
    print(LR_result)

class MotorControllerLR:
    def __init__(self, port, baudrate, unit, resolution=3600 * 4):
        """
        初始化馬達控制器
        :param port: 串口地址 (e.g., '/dev/LRttyUSB')
        :param baudrate: 波特率 (e.g., 9600)
        :param unit: Modbus 地址 (e.g., 0x00)
        :param resolution: 編碼器分辨率
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
        """連接到 Modbus 設備"""
        return self.client.connect()

    def read_encoder_raw(self):
        """讀取編碼器的原始值，並處理回應錯誤"""
        try:
            response_high = self.client.read_holding_registers(0x0007, 1, unit=self.unit)
            response_low = self.client.read_holding_registers(0x0008, 1, unit=self.unit)

            if response_high.isError() or response_low.isError():
                raise ValueError(f"Error reading encoder registers: high={response_high}, low={response_low}")

            position_high = response_high.registers[0]
            position_low = response_low.registers[0]

            encoder_value = (position_high << 16) | position_low
            if encoder_value & 0x80000000:  # 處理負值
                encoder_value = -(0xFFFFFFFF - encoder_value + 1)
            return encoder_value

        except Exception as e:
            print(f"[ERROR] Failed to read encoder raw value: {e}")
            raise

    def close(self):
        """關閉 Modbus 連接"""
        self.client.close()

    @staticmethod
    def float_to_hex(f):
        """將浮點數轉換為 IEEE 754 格式的 16 進位字串"""
        packed = struct.pack('!f', f)
        return packed.hex()

    def read_encoder_raw(self):
        """讀取編碼器的原始值"""
        position_high = self.client.read_holding_registers(0x0007, 1, unit=self.unit).registers[0]
        position_low = self.client.read_holding_registers(0x0008, 1, unit=self.unit).registers[0]
        encoder_value = (position_high << 16) | position_low
        if encoder_value & 0x80000000:  # 處理負值
            encoder_value = -(0xFFFFFFFF - encoder_value + 1)
        return encoder_value

    def move_motor(self, direction, target_circle, set_rpm):
        """
        控制馬達移動
        :param direction: 移動方向 (0x0000 或 0x0001)
        :param target_circle: 目標圈數
        :param set_rpm: 設定轉速
        """
        ieee_circle = self.float_to_hex(target_circle)
        ieee_circle_high = int('0x' + ieee_circle[0:4], 16)
        ieee_circle_low = int('0x' + ieee_circle[4:8], 16)

        ieee_754_rpm = self.float_to_hex(set_rpm)
        ieee_754_rpm_high = int('0x' + ieee_754_rpm[0:4], 16)
        ieee_754_rpm_low = int('0x' + ieee_754_rpm[4:8], 16)

        self.client.write_register(0x004F, 0x0001, unit=self.unit)  # 啟動馬達
        self.client.write_register(0x0053, ieee_754_rpm_high, unit=self.unit)  # 設定轉速高位
        self.client.write_register(0x0054, ieee_754_rpm_low, unit=self.unit)  # 設定轉速低位
        self.client.write_register(0x0050, direction, unit=self.unit)  # 設置方向
        self.client.write_register(0x0055, ieee_circle_high, unit=self.unit)  # 設置目標圈數高位
        self.client.write_register(0x0056, ieee_circle_low, unit=self.unit)  # 設置目標圈數低位
        print(f"Motor {self.unit} moving: direction={direction}, target_circle={target_circle:.4f}")

    def calculate_rpm_by_distance(self, angle_diff):
        """
        根據角度差距動態計算適合的 RPM（轉速）
        :param angle_diff: 角度差距
        :return: RPM 值
        """
        abs_diff = abs(angle_diff)
        if abs_diff > 120:
            return 85
        elif 120 >= abs_diff > 70:
            return 75
        elif 70 >= abs_diff > 60:
            return 65
        elif 60 >= abs_diff > 40:
            return 55
        elif 40 >= abs_diff > 5:
            return 45
        else:
            return 10

    def move_to_angle(self, zero_encoder, target_angle):
        """
        移動到目標角度
        :param zero_encoder: 零點位置
        :param target_angle: 目標角度
        """
        current_encoder = self.read_encoder_raw()
        current_angle = (current_encoder - zero_encoder) * 360.0 / self.resolution
        angle_diff = target_angle - current_angle

        if abs(angle_diff) < 1.0:  # 誤差範圍
            print(f"Motor {self.unit} already at target angle: {target_angle}")
            return

        direction = 0x0000 if angle_diff > 0 else 0x0001
        target_circle = abs(angle_diff / 360.0)
        rpm = self.calculate_rpm_by_distance(angle_diff)

        self.move_motor(direction, target_circle, rpm)

        while True:
            current_encoder = self.read_encoder_raw()
            current_angle = (current_encoder - zero_encoder) * 360.0 / self.resolution
            if abs(current_angle - target_angle) < 1.0:
                self.client.write_register(0x004F, 0x0000, unit=self.unit)  # 停止馬達
                print(f"Reached target angle: {target_angle:.2f}")
                break
            time.sleep(0.05)


    def zero_calibration(self):
        """
        執行歸零校準，尋找左極限與右極限，並計算零點位置
        :return: zero_encoder, left_limit, right_limit
        """
        print("Starting zero calibration...")

        # 左極限
        print("Moving to left limit...")
        left_limit = None
        stable_count = 0
        previous_encoder = None
        while True:
            self.move_motor(0x0000, 1.0, 50)  # 左移
            current_encoder = self.read_encoder_raw()
            if previous_encoder == current_encoder:
                stable_count += 1
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
            self.move_motor(0x0001, 1.0, 50)  # 右移
            current_encoder = self.read_encoder_raw()
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

        return zero_encoder, left_limit, right_limit

# 測試程式入口
if __name__ == "__main__":
    motor = MotorControllerLR(port='/dev/LRttyUSB', baudrate=9600, unit=0x01)

    if not motor.connect():
        print("Cannot connect to motor.")
    else:
        print("Motor connected.")
        try:
            # 載入校準結果
            calibration_result = load_calibration_result("calibration_LR.json")
            if calibration_result:
                zero_encoder = calibration_result["zero_encoder"]
                left_limit = calibration_result["left_limit"]
                right_limit = calibration_result["right_limit"]

                print(f"Calibration loaded: Zero={zero_encoder}, Left={left_limit}, Right={right_limit}")
                # 移動到指定角度，例如 30 度
                motor.move_to_angle(zero_encoder, 30)
            else:
                print("[ERROR] Calibration data not available.")
        except Exception as e:
            print(f"[ERROR] {e}")
        # try:
        #     zero_encoder, left_limit, right_limit = load_calibration_result('/home/robot/wheeltec_ros2/src/rs485_fire/src/calibration_LR.json')
        #     # zero_encoder, down_limit, up_limit = motor.zero_calibration()
        #     # print(f"Calibration complete: Zero={zero_encoder}, Down={down_limit}, Up={up_limit}")
        #     motor.move_to_angle(zero_encoder, 30)  # 移動到 0 度
        # except Exception as e:
        #     print(f"[ERROR] {e}")
        finally:
            motor.close()
            print("Connection closed.")