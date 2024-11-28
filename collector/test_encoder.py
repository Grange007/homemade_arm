import numpy as np

from easyrobot.encoder.unitree_encoder import UnitreeEncoder
from easyrobot.encoder.cybergear_encoder import CybergearEncoder
from easyrobot.encoder.End_effector import EndEffectorEncoder

unitree_encoder = UnitreeEncoder(Unitree_ids = [0, 1, 2])
cybergear_encoder = CybergearEncoder(Cybergear_ids = [1, 2])
# effector_encoder = EndEffectorEncoder(ids = [1, 2, 3, 4, 5, 6, 7, 8], port = '/dev/ttyUSB2', baudrate = 115200)

while True:
    e = encoder.fetch_info()
    print(e)
