import math
import logging
import serial
import struct

logging.basicConfig(filename='Unitree.log', filemode='w', level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def crc16(hex_num):
    crc = '0xffff'
    crc16 = '0xA001'
    test = hex_num.split(' ')

    crc = int(crc, 16)
    crc16 = int(crc16, 16)
    for i in test:
        temp = '0x' + i
        temp = int(temp, 16) 
        crc ^= temp
        for i in range(8):
            if bin(crc)[-1] == '0':
                crc >>= 1
            elif bin(crc)[-1] == '1':
                crc >>= 1
                crc ^= crc16
    crc = hex(crc)
    return crc


class ControlMsg():

    def __init__(self, id=0, status=0, torque=0.0, velocity=0.0, position=0.0, Kp=0.0, Kd=0.0):
        self.id       = id
        self.status   = status
        self.torque   = min(max(torque, -127.99), 127.99)
        self.velocity = min(max(velocity, -804.0), 804.0)
        self.position = min(max(position, -411774), 411774)
        self.Kp       = min(max(Kp, 0.0), 25.599)
        self.Kd       = min(max(Kd, 0.0), 25.599)

    def encode(self):
        id       = self.id
        status   = self.status
        torque   = int(self.torque * 256) + 0x8000
        velocity = int(self.velocity / (2*math.pi) * 256) + 0x8000
        position = int(self.position / (2*math.pi) * 32768) + 0x80000000
        Kp       = int(self.Kp * 1280)
        Kd       = int(self.Kd * 1280)

        data = [id << 4 | status << 1,
                torque >> 8 & 0xff, torque & 0xff,
                velocity >> 8 & 0xff, velocity & 0xff,
                position >> 24 & 0xff, position >> 16 & 0xff, position >> 8 & 0xff, position & 0xff,
                Kp >> 8 & 0xff, Kp & 0xff,
                Kd >> 8 & 0xff, Kd & 0xff]
        msg = "fd ee " + ' '.join(f'{byte:02x}' for byte in data)
        crc = crc16(msg)
        msg += " " + crc[2:] + " " + crc[0:2]
        logging.info("controlMsg: " + msg)
        return msg


class FeedbackMsg():

    def __init__(self, msg=None):
        self.msg = msg

    def decode(self):
        if len(self.msg) != 16:
            logging.error("Invalid feedbackMsg.")
            return False
        
        if self.msg[0] != 0xfd or self.msg[1] != 0xee:
            logging.error("Invalid header.")
            return False
        
        id = self.msg[2] >> 4
        self.id = id

        status = self.msg[2] >> 1 & 0x07
        if status == 0:
            self.status = "Lock"
        elif status == 1:
            self.status = "FOC"
        elif status == 2:
            self.status = "Cali"
        if status >= 3:
            logging.error("Invalid status.")
            return False

        if self.msg[2] & 0x01 != 0:
            logging.error("Invalid")
            return False

        torque = self.msg[3] << 8 | self.msg[4]
        self.torque = (torque - 0x8000) / 256

        velocity = self.msg[5] << 8 | self.msg[6]
        self.velocity = (velocity - 0x8000) * 2*math.pi / 256

        position = self.msg[7] << 24 | self.msg[8] << 16 | self.msg[9] << 8 | self.msg[10]
        self.position = (position - 0x80000000) * 2*math.pi / 32768

        temp = self.msg[11]
        self.temp = temp - 128

        error = self.msg[12] >> 5
        self.error = error != 0

        force = (self.msg[12] & 0x1f) << 7 | self.msg[13] >> 1
        self.force = force

        if self.msg[13] & 0x01 != 0:
            logging.error("Invalid")
            return False

        crc = crc16(' '.join(f'{byte:02x}' for byte in self.msg[:-2]))
        if crc[2:] != f'{self.msg[14]:02x}' or crc[0:2] != f'{self.msg[15]:02x}':
            logging.error("Invalid crc.")
            return False

        return True


class MotorController():

    def __init__(self, port='/dev/ttyUSB0', baudrate=921600, timeout=1):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

    def close(self):
        self.serial.close()

    def clear(self):
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def control(self, send_msg):
        try:
            self.serial.write(bytes.fromhex(send_msg.encode()))
        except:
            logging.error("Failed to send controlMsg.")
            return None
        recv_msg = FeedbackMsg(self.serial.read(16))
        if recv_msg.decode():
            return recv_msg
