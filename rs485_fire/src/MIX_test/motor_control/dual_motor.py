from common.utilities import read_encoder_raw, move_motor
from pymodbus.client.sync import ModbusSerialClient

class DualMotorController:
    def __init__(self, port1, port2, unit1, unit2, baudrate=9600):
        self.client1 = ModbusSerialClient(method='rtu', port=port1, baudrate=baudrate)
        self.client2 = ModbusSerialClient(method='rtu', port=port2, baudrate=baudrate)
        self.unit1 = unit1
        self.unit2 = unit2
        self.resolution = 3600 * 4  # 編碼器分辨率

    def connect(self):
        if not self.client1.connect() or not self.client2.connect():
            raise Exception("Failed to connect to both motors.")
        print("Connected to both motors.")

    def move_motor(self, motor, target_angle, rpm):
        client = self.client1 if motor == 1 else self.client2
        unit = self.unit1 if motor == 1 else self.unit2
        current_encoder = read_encoder_raw(client, unit)
        target_encoder = current_encoder + int(target_angle * self.resolution / 360.0)
        direction = 0x0000 if target_encoder > current_encoder else 0x0001
        target_circle = abs((target_encoder - current_encoder) / self.resolution)
        move_motor(client, unit, direction, target_circle, rpm)

    def close(self):
        self.client1.close()
        self.client2.close()
        print("Connections closed.")
