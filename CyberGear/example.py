import time
import CyberGear

motor_ctrl = CyberGear.MotorCtrl('COM7', 921600, timeout=1)

control_mode_msg = CyberGear.ControlModeMsg()
control_mode_msg.can_id = 1
control_mode_msg.T      = 0.0
control_mode_msg.pos    = 0.0
control_mode_msg.W      = 0.0
control_mode_msg.Kp     = 0.0
control_mode_msg.Ki     = 0.0
while True:
    feedback_msg = motor_ctrl.controlMode(control_mode_msg)
    if feedback_msg != None:
        print("state:", feedback_msg.state)
        print("error:", feedback_msg.error)
        print("cur_can_id:", feedback_msg.cur_can_id)
        print("target_can_id:", feedback_msg.target_can_id)
        print("target_pos:", feedback_msg.target_pos)
        print("target_W:", feedback_msg.target_W)
        print("T:", feedback_msg.T)
        print("temp:", feedback_msg.temp)
    time.sleep(1)

# Close the serial port
motor_ctrl.close()