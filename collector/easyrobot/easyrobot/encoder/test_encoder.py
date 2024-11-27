import numpy as np

from easyrobot.encoder.End_effector import EndEffectorEncoder

encoder = EndEffectorEncoder(ids = [2], port = '/dev/ttyUSB0', baudrate = 115200)

while True:
    e = encoder.fetch_info()
    print(e)
    