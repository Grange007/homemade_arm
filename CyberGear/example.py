import serial
import time
import CyberGear

motor_ctrl = CyberGear.MotorCtrl('/dev/ttyUSB0', 921600, timeout=1)

control_mode_msg = CyberGear.ControlModeMsg()
control_mode_msg.can_id = 1
control_mode_msg.T      = 0.0
control_mode_msg.pos    = 0.0
control_mode_msg.W      = 0.0
control_mode_msg.Kp     = 0.0
control_mode_msg.Ki     = 0.0
feedback_msg = motor_ctrl.controlMode(control_mode_msg)
# print(feedback_msg)
time.sleep(2)

# while True:
#     param_read_msg = CyberGear.ParamReadMsg()
#     param_read_msg.can_id = 1
#     param_read_msg.param  = "mechPos"
#     param_read_msg_r = motor_ctrl.paramRead(param_read_msg)
#     print(param_read_msg_r)
#     time.sleep(1)

# Close the serial port
motor_ctrl.close()