import logging
import time

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

    # 初始化 EndGear 类
    endgear = EndGear(port, baudrate)
    print("初始化完成")
    # 设置 ID
    endgear.set_id(0)
    print("ID 设置为 0")
    
    time.sleep(1)
    # 获取 ID
   # current_id = endgear.get_id()
  #  print(f"当前 ID：{current_id}")

    # 发送数据
    position = 600
    time_duration = 1000
    endgear.send_data(position, time_duration)
    print("数据已发送")

    # 等待一段时间以确保数据发送完成
    time.sleep(2)

    # 获取位置
  #  received_position = endgear.get_position()
  #  print(f"当前位置：{received_position}")

    # 释放扭矩
    endgear.release_torque()
    print("扭矩已释放")

    # 关闭串口
    endgear.close_serial()
    print("串口已关闭")

if __name__ == "__main__":
    print("程序开始执行")
    main()
    print("程序执行完毕")