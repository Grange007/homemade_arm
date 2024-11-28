'''
Angle Encoder Interface.

Author: Hongjie Fang.
This is an example of the angle encoder interface. you can think.
'''

import time
import serial
import numpy as np

from easyrobot.encoder.base import EncoderBase
import CyberGear


class CybergearEncoder(EncoderBase):
    """
    Angle Encoder(s) Interface: receive signals from the angle encoders.
    """
    def __init__(
        self,
        Cybergear_ids,
        sleep_gap = 0.002,
        logger_name: str = "CybergearEncoder",
        shm_name: str = None,
        streaming_freq: int = 30,
        **kwargs 
    ):
        """
        Args:
        - CyberGear_ids: list of int, e.g., [1, 2, ..., 8], the id(s) of the CyberGear encoder(s);
        - sleep_gap: float, optional, default: 0.002, the sleep gap between adjacent write options;
        - logger_name: str, optional, default: "CybergearEncoder", the name of the logger;
        - shm_name: str, optional, default: None, the shared memory name of the angle encoder data, None means no shared memory object;
        - streaming_freq: int, optional, default: 30, the streaming frequency.
        """
        self.Cybergear_ids = CyberGear_ids
        self.Cybergear_ids_map = {}
        for i, id in enumerate(Cybergear_ids):
            self.Cybergear_ids_map[id] = i
        self.Cybergear_init()

        self.ids_num = len(self.Cybergear_ids)

        self.sleep_gap = sleep_gap

        self.init_angle = self.get_angles(ignore_error = False)
        super(CybergearEncoder, self).__init__(
            logger_name = logger_name,
            shm_name = shm_name,
            streaming_freq = streaming_freq
        )

    def Cybergear_init(self):
        self.Cybergear_controller = Cybergear.MotorController('/dev/ttyUSB1', 921600, 1)

    def get_angles(self, ignore_error = False, **kwargs):
        """
        Get the angles of the encoder.

        Args:
        - ignore_error: bool, optional, default: False, whether to ignore the incomplete data error (if set True, then the last results will be used.)
        
        Returns:
        - ret: np.array, the encoder angle results corresponding to the ids array.
        """
        if ignore_error:
            ret = np.copy(self.last_angle).astype(np.float32)
        else:
            ret = np.zeros(self.ids_num).astype(np.float32)

        for id in self.Cybergear_ids:
            control_mode_msg = CyberGear.ControlModeMsg()
            control_mode_msg.can_id   = id
            control_mode_msg.torque   = 0.0
            control_mode_msg.position = 0.0
            control_mode_msg.velocity = 0.0
            control_mode_msg.Kp       = 0.0
            control_mode_msg.Ki       = 0.0
            feedback_msg = self.Cybergear_controller.controlMode(control_mode_msg)
            if feedback_msg is not None:
                ret[self.Cybergear_ids_map[id]] = feedback_msg.position
            else:
                ret[self.Cybergear_ids_map[id]] = 0.0

        if not ignore_error and count != len(ids):
            raise RuntimeError('Failure to receive all encoders, errors occurred in ID {}.'.format(remains))
        self.last_angle = ret
        return ret

    def get_info(self, ignore_error = False, **kwargs):
        """
        Receive signals from the encoders.
        
        Args:
        - ignore_error: bool, optional, default: False, whether to ignore the incomplete data error (if set True, then the last results will be used.)
        
        Returns:
        - ret: np.array, the encoder results corresponding to the ids array.
        """
        return self.get_angles(ignore_error = ignore_error, **kwargs)


    def fetch_info(self):
        if not self.is_streaming:
            self.get_info(ignore_error = True)
        return self.last_angle
