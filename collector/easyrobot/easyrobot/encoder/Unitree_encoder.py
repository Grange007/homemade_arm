'''
Angle Encoder Interface.

Author: Hongjie Fang.
'''

import time
import serial
import numpy as np

from easyrobot.encoder.base import EncoderBase
import easyrobot.encoder.Unitree as Unitree


class UnitreeEncoder(EncoderBase):
    """
    Angle Encoder(s) Interface: receive signals from the angle encoders.
    """
    def __init__(
        self,
        ids,
        port = '/dev/ttyUSB0',
        baudrate = 4000000,
        sleep_gap = 0.002,
        logger_name: str = "UnitreeEncoder",
        shm_name: str = None,
        streaming_freq: int = 30,
        **kwargs 
    ):
        """
        Args:
        - ids: list of int, e.g., [1, 2, ..., 8], the id(s) of the Unitree encoder(s);
        - port, baudrate, (**kwargs): the args of the serial agents;
        - sleep_gap: float, optional, default: 0.002, the sleep gap between adjacent write options;
        - logger_name: str, optional, default: "UnitreeEncoder", the name of the logger;
        - shm_name: str, optional, default: None, the shared memory name of the angle encoder data, None means no shared memory object;
        - streaming_freq: int, optional, default: 30, the streaming frequency.
        """
        self.ids = ids
        self.ids_map = {}
        for i, id in enumerate(ids):
            self.ids_map[id] = i
        self.Unitree_controller = Unitree.MotorController(port, baudrate, 1)
        self.ids_num = len(self.ids)
        self.sleep_gap = sleep_gap
        self.init_angles = self.motor_init()
        super(UnitreeEncoder, self).__init__(
            logger_name = logger_name,
            shm_name = shm_name,
            streaming_freq = streaming_freq
        )
    
    def motor_init(self):
        ret = np.zeros(self.ids_num).astype(np.float32)
        for id in self.ids:
            while True:
                print("Unitree:", id)
                control_msg = Unitree.ControlMsg()
                control_msg.id       = id
                control_msg.status   = 0
                control_msg.torque   = 0.0
                control_msg.position = 0.0
                control_msg.velocity = 0.0
                control_msg.Kp       = 0.0
                control_msg.Kw       = 0.0
                feedback_msg = self.Unitree_controller.control(control_msg)
                if feedback_msg != None:
                    ret[self.ids_map[id]] = (feedback_msg.position / 6.33)
                    break
        self.last_angles = ret
        return ret

    def get_angles(self, ignore_error = False, **kwargs):
        """
        Get the angles of the encoder.

        Args:
        - ignore_error: bool, optional, default: False, whether to ignore the incomplete data error (if set True, then the last results will be used.)
        
        Returns:
        - ret: np.array, the encoder angle results corresponding to the ids array.
        """
        if ignore_error:
            ret = np.copy(self.last_angles).astype(np.float32)
        else:
            ret = np.zeros(self.ids_num).astype(np.float32)

        for id in self.ids:
            control_msg = Unitree.ControlMsg()
            control_msg.id       = id
            control_msg.status   = 1
            control_msg.torque   = 0.0
            control_msg.position = 0.0
            control_msg.velocity = 0.0
            control_msg.Kp       = 0.0
            control_msg.Kw       = 0.0
            feedback_msg = self.Unitree_controller.control(control_msg)
            if feedback_msg != None:
                ret[self.ids_map[id]] = (feedback_msg.position / 6.33)
            else:
                ret[self.ids_map[id]] = self.last_angles[self.ids_map[id]]

        self.last_angles = ret
        return ret

    def get_info(self, ignore_error = False, **kwargs):
        """
        Receive signals from the encoders.
        
        Args:
        - ignore_error: bool, optional, default: False, whether to ignore the incomplete data error (if set True, then the last results will be used.)
        
        Returns:
        - ret: np.array, the encoder results corresponding to the ids array.
        """
        return self.get_angles(ignore_error = ignore_error, **kwargs) - self.init_angles

    def fetch_info(self):
        if not self.is_streaming:
            self.get_info(ignore_error = True)
        return self.last_angles - self.init_angles
