from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time

# 初始化 Modbus 連接
client = ModbusClient(
    method='rtu',       # 通訊模式
    port='/dev/IOttyUSB', # 串口設備
    baudrate=9600,     # 波特率
    stopbits=2,        # 停止位
    bytesize=8,        # 資料位
    timeout=3          # 超時時間
) 

UNIT = 0x01  # Modbus ID

# 定義線圈地址
coil_ball = 0x0006   # 球閥
coil_motor = 0x0003  # 泵浦
coil_ledL = 0x0004   # LED（確認開啟狀態）
coil_ledR = 0x0005

if client.connect():
    print("Connected to Modbus device.")

    while True:
        user_input = input("輸入 o(開) / c(關) / 秒數(定時開啟) / q(退出): ").strip().lower()

        if user_input == 'q':
            print("退出程式")
            break

        try:
            if user_input == 'o':
                # 開啟設備
                client.write_coil(coil_ball, True, unit=UNIT)
                client.write_coil(coil_motor, True, unit=UNIT)
                # client.write_coil(coil_ledL, True, unit=UNIT)
                # client.write_coil(coil_ledR, True, unit=UNIT)
                print("球閥 & 泵浦 已開啟")

            elif user_input == 'c':
                # 關閉設備
                client.write_coil(coil_ball, False, unit=UNIT)
                client.write_coil(coil_motor, False, unit=UNIT)
                # client.write_coil(coil_ledL, False, unit=UNIT)
                # client.write_coil(coil_ledR, False, unit=UNIT)
                # print("球閥 & 泵浦 已關閉")

            else:
                # 解析為秒數
                duration = float(user_input)

                if duration > 0:
                    print(f"啟動球閥 & 泵浦 {duration} 秒...")

                    # 開啟
                    client.write_coil(coil_ball, True, unit=UNIT)
                    client.write_coil(coil_motor, True, unit=UNIT)
                    # client.write_coil(coil_ledL, True, unit=UNIT)
                    # client.write_coil(coil_ledL, True, unit=UNIT)

                    time.sleep(duration)

                    # 關閉
                    client.write_coil(coil_ball, False, unit=UNIT)
                    client.write_coil(coil_motor, False, unit=UNIT)
                    # client.write_coil(coil_ledL, False, unit=UNIT)
                    # client.write_coil(coil_ledR, False, unit=UNIT)

                    print("球閥 & 泵浦 已關閉")
                else:
                    print("[ERROR] 請輸入大於 0 的秒數")

        except ValueError:
            print("[ERROR] 無效輸入，請輸入 'o'、'c' 或秒數")

    # 關閉 Modbus 連線
    client.close()
    print("Connection closed.")

else:
    print("Cannot connect to Modbus device.")
