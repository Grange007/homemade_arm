import os
import glob
import time
import numpy as np

import easyrobot.encoder.Unitree as Unitree
import easyrobot.encoder.Cybergear as Cybergear

class Arm:
    def __init__(self):
        self.positions = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self.time_from_start = [0]
        self.zero_positions = {}
        self.get_trace('./data/motor_example')
        self.Unitree_init('/dev/ttyUSB0', 4000000)
        self.Cybergear_init('/dev/ttyUSB1', 921600)

    def Unitree_init(self, port, baudrate):
        self.Unitree_controller = Unitree.MotorController(port, baudrate, 1)
        for i in range(0, 3):
            while True:
                control_msg = Unitree.ControlMsg()
                control_msg.id       = i
                control_msg.status   = 0
                control_msg.torque   = 0.0
                control_msg.position = 0.0
                control_msg.velocity = 0.0
                control_msg.Kp       = 0.0
                control_msg.Kw       = 0.0
                feedback_msg = self.Unitree_controller.control(control_msg)
                if feedback_msg != None:
                    self.zero_positions[i] = feedback_msg.position
                    break
                time.sleep(0.01)

    def Cybergear_init(self, port, baudrate):
        self.Cybergear_controller = Cybergear.MotorController(port, baudrate, 1)
        for i in range(0, 2):
            print(i+3)
            while True:
                enable_msg = Cybergear.EnableMsg()
                enable_msg.can_id  = i + 1
                enable_msg.host_id = 253
                feedback_msg = self.Cybergear_controller.enable(enable_msg)
                if feedback_msg != None:
                    self.zero_positions[i + 3] = feedback_msg.position
                    break
                time.sleep(0.01)

    def get_trace(self, folder_path):
        files_with_time = []
        for file_path in glob.glob(os.path.join(folder_path, '*.npy')):
            data = np.load(file_path, allow_pickle=True).item()
            if 'time' in data:
                files_with_time.append((file_path, data['time']))
        files_with_time.sort(key=lambda x: x[1])

        for file_path, _ in files_with_time:
            data = np.load(file_path, allow_pickle=True).item()
            if 'time' in data:
                self.time_from_start.append(data['time'])
            vec = []
            if 'unitree_encoder' in data:
                for i in range(len(data['unitree_encoder'])):
                    vec.append(data['unitree_encoder'][i])
            if 'cybergear_encoder' in data:
                for i in range(len(data['cybergear_encoder'])):
                    vec.append(data['cybergear_encoder'][i])
            self.positions.append(vec)

    def start(self):
        print(self.zero_positions)
        for counter in range(1, len(self.time_from_start) - 1):
            print(counter)

            for i in range(0, 3):
                control_msg = Unitree.ControlMsg()
                control_msg.id       = i
                control_msg.status   = 1
                control_msg.torque   = 0.0
                control_msg.position = self.positions[counter][i] * 6.33 + self.zero_positions[i]
                control_msg.velocity = (self.positions[counter + 1][i] - self.positions[counter][i]) / (self.time_from_start[counter + 1] - self.time_from_start[counter]) * 6.33
                control_msg.Kp       = 0.0
                control_msg.Kw       = 0.0
                self.Unitree_controller.control(control_msg)
                time.sleep(0.01)

            for i in range(0, 2):
                control_mode_msg = Cybergear.ControlModeMsg()
                control_mode_msg.can_id   = i + 1
                control_mode_msg.torque   = 0.0
                control_mode_msg.position = self.positions[counter][i + 3] + self.zero_positions[i + 3]
                control_mode_msg.velocity = (self.positions[counter + 1][i + 3] - self.positions[counter][i + 3]) / (self.time_from_start[counter + 1] - self.time_from_start[counter]) * 6.33
                control_mode_msg.Kp       = 0.0
                control_mode_msg.Ki       = 0.0
                self.Cybergear_controller.controlMode(control_mode_msg)
                time.sleep(0.01)

            time.sleep((self.time_from_start[counter + 1] - self.time_from_start[counter]) / 100)


if __name__ == '__main__':
    arm = Arm()
    arm.start()