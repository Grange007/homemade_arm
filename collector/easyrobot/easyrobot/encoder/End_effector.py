'''
Angle Encoder Interface.

Author: Hongjie Fang.
This is an example of the angle encoder interface. you can think.
'''

import copy
import time
import serial
import numpy as np

from easyrobot.encoder.base import EncoderBase


class EndEffectorEncoder(EncoderBase):
    """
    Angle Encoder(s) Interface: receive signals from the angle encoders.
    """
    def __init__(
        self,
        ids,
        port,
        baudrate = 115200,
        sleep_gap = 0.05,
        logger_name: str = "EndEffector Encoder",
        shm_name: str = None, 
        streaming_freq: int = 20,
        **kwargs 
    ):
        """
        Args:
        - ids: list of int, e.g., [1, 2, ..., 8], the id(s) of the desired encoder(s);
        - port, baudrate, (**kwargs): the args of the serial agents;
        - sleep_gap: float, optional, default: 0.05, the sleep gap between adjacent write options;
        - logger_name: str, optional, default: "AngleEncoder", the name of the logger;
        - shm_name: str, optional, default: None, the shared memory name of the angle encoder data, None means no shared memory object;
        - streaming_freq: int, optional, default: 30, the streaming frequency.
        """
        self.ids = ids
        self.ids_num = len(ids)
        self.ids_map = {}
        for i, id in enumerate(ids):
            self.ids_map[id] = i
        self.sleep_gap = sleep_gap
        self.ser = serial.Serial(port, baudrate = baudrate, **kwargs)
        if not self.ser.is_open:
            raise RuntimeError('Fail to open the serial port, please check your settings again.')
        self.ser.flushInput()
        self.ser.flushOutput()
        self.last_angle = self.get_angles(ignore_error = False)
        super(EndEffectorEncoder, self).__init__(
            logger_name = logger_name, 
            shm_name = shm_name, 
            streaming_freq = streaming_freq
        )

    def get_angles(self, ignore_error = False, **kwargs):
        """
        Get the angles of the encoder.

        Args:
        - ignore_error: bool, optional, default: False, whether to ignore the incomplete data error (if set True, then the last results will be used.)
        
        Returns:
        - ret: np.array, the encoder angle results corresponding to the ids array.
        """
        self.ser.flushInput()
        ids = copy.deepcopy(self.ids)
        for i in ids:
            send_data = f"#{i:03d}PRAD!\n"
            self.ser.write(send_data.encode('utf-8'))
            time.sleep(self.sleep_gap)
            
        rec_data = ""
        count = 0
        results = []
        while count < self.ids_num:
            char = self.ser.read().decode('utf-8', errors='ignore')  # 逐个字符读取
            rec_data += char
            if char == '!':  # 遇到 '!' 停止读取并保存结果
                results.append(rec_data)
                rec_data = ""  # 重置 rec_data 以便读取下一个数据块
                count += 1

# 读取完成后，results 列表中包含了 self.ids_num 个数据块
        angles = {}
        for data in results:
            try:
                parts = data.strip().split('P')
                id_str = parts[0][1:]  # 去掉 '#' 并获取 ID 部分
                angle_str = parts[1][:-1]  # 去掉 '!' 并获取角度部分
                id = int(id_str)
                angle = int(angle_str)
                angle = (angle-500)/2000*270
                angles[id] = angle
            except (IndexError, ValueError) as e:
                if not ignore_error:
                    raise RuntimeError(f"Error parsing data: {data}") from e
        
        if not ignore_error and count != len(ids):
            raise RuntimeError('Failure to receive all encoders, errors occurred in ID {}.'.format(remains))
        self.last_angle = angles
        return angles
    
    def release_F(self, id):
        self.ser.flushInput()
        self.ser.write(f"#{id:03d}PULK!\n".encode('utf-8'))
        rec_data = ""
        while True:
            char = self.ser.read().decode('utf-8')  # 逐个字符读取
            rec_data += char
            if char == '!':  # 遇到 '!' 停止读取并保存结果
                break
        if rec_data != f"#OK!":
            raise RuntimeError(f"Error releasing F: {rec_data}")
            return False
        else:
            return True
        
    def recover_F(self, id):
        self.ser.flushInput()
        self.ser.write(f"#{id:03d}PULR!\n".encode('utf-8'))
        rec_data = ""
        while True:
            char = self.ser.read().decode('utf-8')  # 逐个字符读取
            rec_data += char
            if char == '!':  # 遇到 '!' 停止读取并保存结果
                break
        if rec_data != f"#OK!\n":
            raise RuntimeError(f"Error releasing F: {rec_data}")
            return False
        else:
            return True
        
    def get_info(self, ignore_error = False, **kwargs):
        """
        Receive signals from the encoders.
        
        Args:
        - ignore_error: bool, optional, default: False, whether to ignore the incomplete data error (if set True, then the last results will be used.)
        
        Returns:
        - ret: np.array, the encoder results corresponding to the ids array.
        """
        for i in self.ids:
            self.release_F(i)
        return self.get_angles(ignore_error = ignore_error, **kwargs)


    def fetch_info(self):
        if not self.is_streaming:
            self.get_info(ignore_error = True)
        return self.last_angle
