import numpy as np
import sys
sys.path.append('f:/清清电子资料库/学习相关/大三上/电子系统设计/code/collector/easyrobot')

from easyrobot.encoder.End_effector import EndEffectorEncoder

encoder = EndEffectorEncoder(ids = [2], port = 'COM4', baudrate = 115200)

while True:
    e = encoder.fetch_info()
    print(e)
    