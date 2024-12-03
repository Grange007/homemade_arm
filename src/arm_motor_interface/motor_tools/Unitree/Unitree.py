import math
import logging
import serial
import struct

import sys
sys.path.append('./lib')
import UnitreeMotorSDK

logging.basicConfig(filename='Unitree.log', filemode='w', level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


class ControlMsg():

    def __init__(self, id=0, status=0, torque=0.0, velocity=0.0, position=0.0, Kp=0.0, Kw=0.0):
        self.id       = id
        self.status   = status
        self.torque   = min(max(torque, -127.99), 127.99)
        self.velocity = min(max(velocity, -804.0), 804.0)
        self.position = min(max(position, -411774), 411774)
        self.Kp       = min(max(Kp, 0.0), 25.599)
        self.Kw       = min(max(Kw, 0.0), 25.599)

    def encode(self):
        msg = UnitreeMotorSDK.MotorCmd()
        msg.id   = self.id
        msg.mode = self.status
        msg.T    = self.torque
        msg.W    = self.velocity
        msg.Pos  = self.position
        msg.K_P  = self.Kp
        msg.K_W  = self.Kw
        return msg


class FeedbackMsg():

    def __init__(self, msg=None):
        self.msg = msg

    def decode(self):
        if self.msg.correct == False:
            logging.error("Invalid feedbackMsg.")
            return False
        
        self.id       = self.msg.motor_id
        self.status   = self.msg.mode
        self.torque   = self.msg.T
        self.velocity = self.msg.W
        self.position = self.msg.Pos
        self.temp     = self.msg.Temp
        self.error    = self.msg.MError
        self.force    = self.msg.footForce
        return True


class MotorController():

    def __init__(self, port='/dev/ttyUSB0', baudrate=4000000, timeout=1):
        self.serial = UnitreeMotorSDK.SerialPort(port)

    def control(self, send_msg):
        send_msg = send_msg.encode()
        recv_msg = UnitreeMotorSDK.MotorData()
        try:
            self.serial.sendRecv(send_msg, recv_msg)
        except:
            logging.error("Failed to send controlMsg.")
            return None
        recv_msg = FeedbackMsg(recv_msg)
        if recv_msg.decode():
            return recv_msg
        else:
            return None
