from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException

import time

client = ModbusClient(
    method='rtu',
    port='/dev/IOttyUSB',
    baudrate=9600,
    stopbits=2,
    bytesize=8,
    timeout=3
)

UNIT = 0x01
SENSORS = {
    '前安全感測器': 0x0002,
    '後安全感測器': 0x0000,
    '水位感測器':   0x0004,
}

if not client.connect():
    print("無法連接到設備")
    exit()

try:
    while True:
        for name, address in SENSORS.items():
            result = client.read_discrete_inputs(address, count=1, unit=UNIT)
            if not result.isError():
                sensor_status = result.bits[0]
                if not sensor_status:  # False 表示感測器有反應
                    print(f"{name} 有反應！")
            else:
                print(f"{name} 讀取失敗")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("手動中止程式")

finally:
    client.close()
    exit()
