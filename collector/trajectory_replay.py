import numpy as np
import glob
import os
import time
import serial

class EndEffectorTra:
    def __init__(self, file_path, ids,frequency,servo_port):
        """
        初始化 EndEffectorTra 类。

        参数:
        - file_path: str, 读取文件的路径。
        - ids: list, 用于轨迹复现的 ID 列表。
        """
        self.file_path = file_path
        self.ids = ids
        self.npy_list = self.load_all_npy()
        self.frequency = frequency 
        self.sleepgap = 1/self.frequency
        self.ser = serial.Serial(servo_port, baudrate=115200, timeout=1)
    def load_all_npy(self):
        """
        读取 file_path 目录下的所有 .npy 文件并返回数据列表。

        返回:
        - list, 包含所有 norm_stats 的字典列表。
        """
        # 获取所有 .npy 文件路径
        npy_files = glob.glob(os.path.join(self.file_path, '*.npy'))
        
        # 按文件名中的数字排序
        npy_files.sort(key=lambda x: int(os.path.splitext(os.path.basename(x))[0]))

        norm_stats_list = []
        for npy_file in npy_files:
            norm_stats = np.load(npy_file, allow_pickle=True).item()
            norm_stats_list.append(norm_stats)
        
        return norm_stats_list

    def reproduce_trajectory(self):
        """
        进行轨迹复现并打印轨迹数据。
        """
        ids_num = len(self.ids)
        sleepgap1 = self.sleepgap*1000
        sleepgap1 = int(sleepgap1)
        for npy in self.npy_list:
            angles = npy['servo_encoder']
            pos = angles/270*2000+500
            pos = pos.astype(int)
            print('POS:',pos)
            
            for i in range(ids_num):
                send_data=''
                send_data += f"#{self.ids[i]:03d}P{pos[i]:04d}T{sleepgap1:04d}!"
                print(send_data)
                self.ser.write(send_data.encode('utf-8')) 
                time.sleep(self.sleepgap)
            # 示例：打印给定 ID 的轨迹数据
            
    def close_servo(self):
        self.ser.close()

# 示例使用
file_path = '/home/grange/Program/act_arm/collector/data/task2/scene1'
ids = [1, 2]

port = '/dev/ttyUSB0'
end_effector_tra = EndEffectorTra(file_path, ids, 100, port)
end_effector_tra.reproduce_trajectory()