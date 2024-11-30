import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

import sys
import os
cur_dir = os.path.dirname(os.path.abspath(__file__))
Cybergear_dir = os.path.abspath(os.path.join(cur_dir, '..', 'motor_tools', 'Cybergear'))
if Cybergear_dir not in sys.path:
    sys.path.append(Cybergear_dir)
import Cybergear
Unitree_dir = os.path.abspath(os.path.join(cur_dir, '..', 'motor_tools', 'Unitree'))
if Unitree_dir not in sys.path:
    sys.path.append(Unitree_dir)
import Unitree


class MotorJointStatePublisher:
    def __init__(self) -> None:
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_state = JointState()
        self.zero_positions = {}
        self.Unitree_init('/dev/ttyUSB0', 4000000)
        self.Cybergear_init('/dev/ttyUSB1', 921600)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def Unitree_init(self, port, baudrate):
        self.Unitree_controller = Unitree.MotorController(port, baudrate, 1)
        for i in range(0, 3):
            while True:
                control_msg = Unitree.ControlMsg()
                control_msg.id       = i
                control_msg.status   = 0
                control_msg.torque   = 0.0
                control_msg.position = 0.0
                control_msg.velocity = 0.0
                control_msg.Kp       = 0.0
                control_msg.Kw       = 0.0
                feedback_msg = self.Unitree_controller.control(control_msg)
                if feedback_msg != None:
                    self.zero_positions[i] = feedback_msg.position
                    break

    def Cybergear_init(self, port, baudrate):
        self.Cybergear_controller = Cybergear.MotorController(port, baudrate, 1)
        for i in range(0, 2):
            while True:
                enable_msg = Cybergear.EnableMsg()
                enable_msg.can_id  = i + 1
                enable_msg.host_id = 253
                feedback_msg = self.Cybergear_controller.enable(enable_msg)
                if feedback_msg != None:
                    self.zero_positions[i + 3] = feedback_msg.position
                    break

    def timer_callback(self, event):
        for i in range(0, 3):
            control_msg = Unitree.ControlMsg()
            control_msg.id       = i
            control_msg.status   = 1
            control_msg.torque   = 0.0
            control_msg.position = 0.0
            control_msg.velocity = 0.0
            control_msg.Kp       = 0.0
            control_msg.Kw       = 0.0
            feedback_msg = self.Unitree_controller.control(control_msg)
            if feedback_msg != None:
                if feedback_msg.position - self.zero_positions[i] < 0:
                    self.joint_angles[i] = -(((-feedback_msg.position + zero_positions[i]) / 6.33) % 6.28)
                else:
                    self.joint_angles[i] = ((feedback_msg.position - zero_positions[i]) / 6.33) % 6.28
        
        for i in range(0, 2):
            control_mode_msg = Cybergear.ControlModeMsg()
            control_mode_msg.can_id   = i + 1
            control_mode_msg.torque   = 0.0
            control_mode_msg.position = 0.0
            control_mode_msg.velocity = 0.0
            control_mode_msg.Kp       = 0.0
            control_mode_msg.Ki       = 0.0
            feedback_msg = self.Cybergear_controller.controlMode(control_mode_msg)
            if feedback_msg != None:
                if feedback_msg.position - self.zero_positions[i + 3] < 0:
                    self.joint_angles[i + 3] = -((-feedback_msg.position + self.zero_positions[i + 3]) % 6.28)
                else:
                    self.joint_angles[i + 3] = (feedback_msg.position - self.zero_positions[i + 3]) % 6.28

        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.name = self.joint_names
        self.joint_state.position = self.joint_angles
        self.joint_state_pub.publish(self.joint_state)


if __name__ == '__main__':
    rospy.init_node('motor_joint_state_publisher')
    motor_joint_state_publisher = MotorJointStatePublisher()
    rospy.spin()
