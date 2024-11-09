import time
import CyberGear

motor_ctrl = CyberGear.MotorCtrl('COM7', 921600, timeout=1)


enable_msg = CyberGear.EnableMsg()
enable_msg.can_id  = 1
enable_msg.host_id = 253
feedback_msg = motor_ctrl.enable(enable_msg)
if feedback_msg != None:
    print("state:   ", feedback_msg.state)
    print("error:   ", feedback_msg.error)
    print("can_id:  ", feedback_msg.can_id)
    print("host_id: ", feedback_msg.host_id)
    print("position:", feedback_msg.position)
    print("velocity:", feedback_msg.velocity)
    print("torque:  ", feedback_msg.torque)
    print("temp:    ", feedback_msg.temp)
time.sleep(0.5)

control_mode_msg = CyberGear.ControlModeMsg()
control_mode_msg.can_id   = 1
control_mode_msg.torque   = 0.0
control_mode_msg.position = 3.0
control_mode_msg.velocity = 0.0
control_mode_msg.Kp       = 0.1
control_mode_msg.Ki       = 0.0
feedback_msg = motor_ctrl.controlMode(control_mode_msg)
if feedback_msg != None:
    print("state:   ", feedback_msg.state)
    print("error:   ", feedback_msg.error)
    print("can_id:  ", feedback_msg.can_id)
    print("host_id: ", feedback_msg.host_id)
    print("position:", feedback_msg.position)
    print("velocity:", feedback_msg.velocity)
    print("torque:  ", feedback_msg.torque)
    print("temp:    ", feedback_msg.temp)
time.sleep(3)

disable_msg = CyberGear.DisableMsg()
disable_msg.can_id  = 1
disable_msg.host_id = 253
disable_msg.fault   = False
feedback_msg = motor_ctrl.disable(disable_msg)
if feedback_msg != None:
    print("state:   ", feedback_msg.state)
    print("error:   ", feedback_msg.error)
    print("can_id:  ", feedback_msg.can_id)
    print("host_id: ", feedback_msg.host_id)
    print("position:", feedback_msg.position)
    print("velocity:", feedback_msg.velocity)
    print("torque:  ", feedback_msg.torque)
    print("temp:    ", feedback_msg.temp)

# Close the serial port
motor_ctrl.close()