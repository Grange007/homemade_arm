import logging
import time
import serial

# 假设 EndGear 类已经定义在 EndGear.py 文件中
from EndGear import EndGear

def main():
    logging.basicConfig(
        level=logging.DEBUG,
        filename='example.log',
        filemode='w',
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    port = "COM4"
    baudrate = 115200
    shared_ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)

    # 初始化 EndGear 类
    endgear1 = EndGear(1,shared_ser)
    endgear1.open_serial()
    endgear2 = EndGear(2,shared_ser)
    endgear2.open_serial()
    print("初始化完成")

    time.sleep(1)
    # 获取 ID
    current_id1 = endgear1.get_id()
    time.sleep(0.5)
    current_id2 = endgear2.get_id()
    print(f"当前 ID：{current_id1}")
    print(f"当前 ID：{current_id2}")

    # 发送数据
    position = 600
    time_duration = 1000
    endgear1.send_data(position, time_duration)
    time.sleep(0.5)
    endgear2.send_data(position, time_duration)
    print("数据已发送")

    # 等待一段时间以确保数据发送完成
    time.sleep(2)

    # 获取位置
    received_position1 = endgear1.get_position()
    time.sleep(0.5)
    received_position2 = endgear2.get_position()
    print(f"当前位置1：{received_position1}")
    print(f"当前位置2：{received_position2}")

    # 释放扭矩
    endgear1.release_torque()
    print("扭矩已释放")
    time.sleep(0.5)
    # 关闭串口
    endgear2.close_serial()
    print("串口已关闭")

if __name__ == "__main__":
    print("程序开始执行")
    main()

    print("程序执行完毕")