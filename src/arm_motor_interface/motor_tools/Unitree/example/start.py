import time
import sys
sys.path.append('../lib')
import UnitreeMotorSDK

serial = UnitreeMotorSDK.SerialPort('/dev/ttyUSB0')
cmd = UnitreeMotorSDK.MotorCmd()
data = UnitreeMotorSDK.MotorData()

for _ in range(500):
    cmd.id = 0
    cmd.mode = 1
    cmd.T = 0.0
    cmd.W = 3.14 * 6.33
    cmd.Pos = 0.0
    cmd.K_P = 0.0
    cmd.K_W = 0.05
    serial.sendRecv(cmd, data)
    if data.correct:
        print("id:   ", data.motor_id)
        print("mode: ", data.mode)
        print("T:    ", data.T)
        print("W:    ", data.W)
        print("Pos:  ", data.Pos)
    time.sleep(0.01)