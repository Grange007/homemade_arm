import logging
import serial

logging.basicConfig(filename='EndGear.log', filemode='w', level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
#末端执行器的功能：1.设置转动位置(以多长的时间转到哪)
#2.获取当前位置(500-2500对应0-270°)
#3.释放扭力
#4.读取和设置ID
class EndGear:
    def __init__(self,id,ser):
        self.ser = ser
        self.id = id  # 初始化 id 变量
        logging.info(f"已初始化 EndGear 类，ID：{self.id}")
       
    def open_serial(self):
        if self.ser.is_open:
            logging.info("串口已经是打开状态")
            return
        try:
            self.ser.open()
            logging.info("串口已打开")
        except Exception as e:
            logging.error(f"未成功打开：{e}")

    def readdata(self):	
        data = ""
        while True:
            char = self.ser.read().decode('utf-8')  # 逐个字符读取
            data += char
            if char == '!':  # 遇到 '!' 停止读取
                break 
        return data       
       
    def close_serial(self):
        if self.ser.is_open:
            self.ser.close()
            logging.info("串口已关闭")

    def send_data(self, position, time):
        if position < 500 or position > 2500:
            logging.error("位置超出范围")
            return
        command_to_send = f"#{self.id:03d}P{int(position):04d}T{int(time):04d}!\n"
        self.ser.write(command_to_send.encode('utf-8'))
        logging.info("已发送指令：" + command_to_send)

    def get_position(self):
        command_to_send = f"#{self.id:03d}PRAD!\n"
        self.ser.write(command_to_send.encode('utf-8'))
        logging.info("已发送指令：" + command_to_send)
        data = self.readdata()
        logging.info("已接收数据：" + data)
        return int(data[5:8])

    def get_id(self):
        command_to_send = f"#{self.id:03d}PID!\n"
        self.ser.write(command_to_send.encode('utf-8'))
        logging.info("已发送指令：" + command_to_send)
        data = self.readdata()
        logging.info("已接收数据：" + data)
        logging.info(f"当前 ID：{data[1:4]}")
        self.id = int(data[1:4])  # 更新 id 变量
        return self.id

    def set_id(self, id):
        if id < 0 or id > 254:
            logging.error("ID超出范围")
            return
        command_to_send = f"#{self.id:03d}PID{int(id):03d}!\n"
        self.ser.write(command_to_send.encode('utf-8'))
        logging.info("已发送指令：" + command_to_send)
        self.id = id  # 更新 id 变量

    def release_torque(self):
        command_to_send = f"#{self.id:03d}PULK!\n"
        self.ser.write(command_to_send.encode('utf-8'))
        logging.info("已发送指令：" + command_to_send)
        data = self.readdata()
        logging.info("已接收数据：" + data)
    

