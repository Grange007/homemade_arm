import math
import logging
import serial


PARAMETERS = {
    "run_mode":      {"index": 0x7005, "format": "u8", "mod": "rw"},
    "iq_ref":        {"index": 0x7006, "format": "f",  "mod": "rw"},
    "spd_ref":       {"index": 0x700A, "format": "f",  "mod": "rw"},
    "limit_torque":  {"index": 0x700B, "format": "f",  "mod": "rw"},
    "cur_kp":        {"index": 0x7010, "format": "f",  "mod": "rw"},
    "cur_ki":        {"index": 0x7011, "format": "f",  "mod": "rw"},
    "cur_filt_gain": {"index": 0x7014, "format": "f",  "mod": "rw"},
    "loc_ref":       {"index": 0x7016, "format": "f",  "mod": "rw"},
    "limit_spd":     {"index": 0x7017, "format": "f",  "mod": "rw"},
    "limit_cur":     {"index": 0x7018, "format": "f",  "mod": "rw"},
    "mechPos":       {"index": 0x7019, "format": "f",  "mod": "r"},
    "mechVel":       {"index": 0x701A, "format": "f",  "mod": "r"},
}


class ControlModeMsg():

    def __init__(self, can_id=1, T=0.0, pos=0.0, W=0.0, Kp=0.0, Kd=0.0):
        self.can_id = can_id
        self.T      = min(max(T, -12.0), 12.0)
        self.pos    = min(max(pos, -4*math.pi), 4*math.pi)
        self.W      = min(max(W, -30.0), 30.0)
        self.Kp     = min(max(Kp, 0.0), 500.0)
        self.Kd     = min(max(Kd, 0.0), 5.0)

    def encode(self):
        type   = 0x01
        T      = int((self.T + 12.0)/24 * 0xffff)
        can_id = self.can_id << 3 | 0x04
        len    = 0x08
        pos    = int((self.pos + 4*math.pi)/(8*math.pi)) * 0xffff)
        W      = int((self.W + 30.0)/60 * 0xffff)
        Kp     = int(self.Kp / 500 * 0xffff)
        Kd     = int(self.Kd / 5 * 0xffff)

        data = [type << 3 | T >> 13, T >> 5 & 0xff, (T & 0x1f) << 3 | can_id >> 8, can_id & 0xff,
                len, pos >> 8, pos & 0xff, W >> 8, W & 0xff, Kp >> 8, Kp & 0xff, Kd >> 8, Kd & 0xff]
        msg = "41 54 " + ' '.join(f'{byte:02X}' for byte in data) + " 0d 0a"
        return msg


class FeedbackMsg():

    def __init__(self, msg=None):
        self.msg = msg

    def decode(self):
        if self.msg[0] != 0x41 and self.msg[1] != 0x54 and self.msg[15] != 0x0d and self.msg[16] != 0x0a:
            logging.info("Invalid message.")
            return False
        if self.msg[6] != 0x08:
            logging.info("Invalid length.")
            return False
        if self.msg[2] >> 3 != 0x02:
            logging.info("Invalid type.")
            return False
        if self.msg[5] & 0x07 != 0x04:
            logging.info("Invalid id.")
            return False

        state = msg[2] >> 1 & 0x03
        if state == 0:
            self.state = "Reset"
        elif state == 1:
            self.state = "Cali"
        elif state == 2:
            self.state = "Run"
        else
            logging.info("Invalid state.")
            return False

        error = (msg[2] & 0x01) << 5 | msg[3] >> 3
        self.error = error != 0

        can_id = (msg[3] & 0x07) << 5 | msg[4] >> 3
        if can_id != (msg[4] & 0x07) << 5 | msg[5] >> 3:
            logging.info("Invalid can id.")
            return False
        self.can_id = can_id

        target_pos = msg[8] << 8 | msg[7]
        self.target_pos = (target_pos / 0xffff * 8*math.pi) - 4*math.pi

        target_W = msg[10] << 8 | msg[9]
        self.target_W = (target_W / 0xffff * 60) - 30

        T = msg[12] << 8 | msg[11]
        self.T = (T / 0xffff * 24) - 12

        temp = msg[14] << 8 | msg[13]
        self.temp = temp / 0xffff * 125

        return True


class EnableMsg():

    def __init__(self, can_id=1):
        self.can_id = can_id

    def encode(self):
        return None


class DisableMsg():
    
    def __init__(self, can_id=1):
        self.can_id = can_id

    def encode(self):
        return None


class ParamReadMsg():

    def __init__(self, can_id=1, param=None, msg=None):
        self.can_id = can_id
        self.param  = param
        self.msg    = msg

    def encode(self):
        type   = 0x11
        can_id = self.can_id << 3 | 0x04
        index  = PARAMETERS[self.param]["index"]
        len    = 0x80

        data = [type << 3, 0x00, can_id >> 8, can_id & 0xff,
                len, index >> 8, index & 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg = "41 54 " + ' '.join(f'{byte:02X}' for byte in data) + " 0d 0a"
        return msg

    def decode(self):
        if self.msg[0] != 0x41 and self.msg[1] != 0x54 and self.msg[15] != 0x0d and self.msg[16] != 0x0a:
            logging.info("Invalid message.")
            return False
        if self.msg[6] != 0x80: 
            logging.info("Invalid length.")
            return False
        if self.msg[2] >> 3 != 0x12:
            logging.info("Invalid type.")
            return False
        if self.msg[5] & 0x07 != 0x04:
            logging.info("Invalid id.")
            return False
        
        can_id = (msg[4] & 0x07) << 5 | msg[5] >> 3:
        self.can_id = can_id

        param = msg[8] << 8 | msg[7]
        if param not in PARAMETERS["index"]:
            logging.info("Invalid param.")
            return False
        self.param = param

        if msg[9] != 0x00 or msg[10] != 0x00
            logging.info("Invalid data.")
            return False
        
        value = msg[14] << 24 | msg[13] << 16 | msg[12] << 8 | msg[11]
        elif self.param == 0x7005:
            if value == 0:
                self.value = "control mode"
            elif value == 1:
                self.value = "position mode"
            elif value == 2:
                self.value = "speed mode"
            elif value == 3:
                self.value = "current mode"
            else:
                logging.info("Invalid value.")
                return False
        elif self.param == 0x7019:
            self.value = struct.unpack('f', struct.pack('I', value))[0]
        elif self.param == 0x701A:
            self.value = struct.unpack('f', struct.pack('I', value))[0]

        return True


class ParamWriteMsg():

    def __init__(self, can_id=1, param=None, value=None):
        self.can_id = id
        self.param  = param
        self.value  = value

    def encode(self):
        return None


class MotorCtrl():

    def __init__(self, serial):
        self.serial = serial
        self.serial.write(bytes.fromhex('41 54 2b 41 54 0d 0a'))

    def controlMode(self, send_msg):
        try:
            self.serial.write(bytes.fromhex(send_msg.encode()))
        except:
            logging.info("Failed to send controlModeMsg.")

        recv_msg = FeedbackMsg(self.serial.read(17))
        if (recv_msg.decode())
            return recv_msg

    def paramRead(self, send_msg)
        try:
            self.serial.write(bytes.fromhex(send_msg.encode()))
        except:
            logging.info("Failed to send paramReadMsg.")
        
        recv_msg = ParamReadMsg(self.serial.read(17))
        if (recv_msg.decode())
            return recv_msg