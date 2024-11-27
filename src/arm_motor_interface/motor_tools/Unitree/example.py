import time
import Unitree

motor_controller = Unitree.MotorController('/dev/ttyUSB0', 921600, timeout=1)

control_msg = Unitree.ControlMsg()
control_msg.id       = 0
control_msg.status   = 1
control_msg.torque   = 0.0
control_msg.position = 0.0
control_msg.velocity = 0.0
control_msg.Kp       = 0.0
control_msg.Kw       = 0.0
feedback_msg = motor_controller.control(control_msg)
if feedback_msg != None:
    print("id:      ", feedback_msg.id)
    print("status:  ", feedback_msg.status)
    print("torque:  ", feedback_msg.torque)
    print("position:", feedback_msg.position)
    print("velocity:", feedback_msg.velocity)
    print("Kp:      ", feedback_msg.Kp)
    print("Kw:      ", feedback_msg.Kw)

# Close the serial port
motor_controller.close()