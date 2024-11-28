import numpy as np

from easyrobot.encoder.unitree_encoder import UnitreeEncoder
from easyrobot.encoder.cybergear_encoder import CybergearEncoder
from easyrobot.encoder.End_effector import EndEffectorEncoder

# unitree_encoder = UnitreeEncoder(Unitree_ids = [0, 1, 2])
# cybergear_encoder = CybergearEncoder(Cybergear_ids = [1, 2])
effector_encoder = EndEffectorEncoder(ids = [1, 2], port = '/dev/ttyUSB0', baudrate = 115200)

while True:
    effector_encoder.release_F(1)
    effector_encoder.release_F(2)
    e = effector_encoder.fetch_info()
    print(e)
