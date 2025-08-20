from pymodbus.client.sync import ModbusSerialClient

def read_encoder_raw(client, unit):
    """從編碼器讀取原始值，並處理負值。"""
    position_high = client.read_holding_registers(0x0007, 1, unit=unit).registers[0]
    position_low = client.read_holding_registers(0x0008, 1, unit=unit).registers[0]
    encoder_value = (position_high << 16) | position_low
    if encoder_value & 0x80000000:
        encoder_value = -(0xFFFFFFFF - encoder_value + 1)
    return encoder_value

def move_motor(client, unit, direction, target_circle, set_rpm):
    """控制馬達移動到指定目標圈數。"""
    import struct
    ieee_circle = struct.pack('!f', target_circle).hex()
    ieee_rpm = struct.pack('!f', set_rpm).hex()
    client.write_register(0x004F, 0x0001, unit=unit)  # 啟動馬達
    client.write_register(0x0053, int(ieee_rpm[:4], 16), unit=unit)  # 轉速高位
    client.write_register(0x0054, int(ieee_rpm[4:], 16), unit=unit)  # 轉速低位
    client.write_register(0x0050, direction, unit=unit)  # 設定方向
    client.write_register(0x0055, int(ieee_circle[:4], 16), unit=unit)  # 圈數高位
    client.write_register(0x0056, int(ieee_circle[4:], 16), unit=unit)  # 圈數低位
    print(f"Motor {unit}: direction={direction}, target_circle={target_circle}, rpm={set_rpm}")
