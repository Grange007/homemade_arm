import rospy
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

import sys
import os
cur_dir = os.path.dirname(os.path.abspath(__file__))
CyberGear_dir = os.path.abspath(os.path.join(cur_dir, '..', 'motor_tools', 'CyberGear'))
if CyberGear_dir not in sys.path:
    sys.path.append(CyberGear_dir)
import CyberGear


class MotorJointStatePublisher:
    def __init__(self) -> None:
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.unitree_joint_state_sub = rospy.Subscriber('unitree_joint_states', JointState, self.unitree_joint_state_callback)
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.joint_state = JointState()
        self.CyberGear_init()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def CyberGear_init(self):
        self.cybergear_motor_controller = CyberGear.MotorController('/dev/ttyUSB0', 921600, timeout=1)
        # joint 4
        enable_msg_1 = CyberGear.EnableMsg()
        enable_msg_1.can_id  = 1
        enable_msg_1.host_id = 253
        self.cybergear_motor_controller.enable(enable_msg_1)
        # joint 5
        enable_msg_2 = CyberGear.EnableMsg()
        enable_msg_2.can_id  = 2
        enable_msg_2.host_id = 253
        self.cybergear_motor_controller.enable(enable_msg_2)

    def unitree_joint_state_callback(self, msg):
        self.joint_angles[0] = msg.position[0]
        self.joint_angles[1] = msg.position[1]
        self.joint_angles[2] = msg.position[2]

    def timer_callback(self, event):
        # joint 4
        control_mode_msg_1 = CyberGear.ControlModeMsg()
        control_mode_msg_1.can_id   = 1
        control_mode_msg_1.torque   = 0.0
        control_mode_msg_1.position = 0.0
        control_mode_msg_1.velocity = 0.0
        control_mode_msg_1.Kp       = 0.0
        control_mode_msg_1.Ki       = 0.0
        feedback_msg_1 = self.cybergear_motor_controller.controlMode(control_mode_msg_1)
        if feedback_msg_1 is not None:
            if feedback_msg_1.position < 0:
                self.joint_angles[3] = -((-feedback_msg_1.position) % (2*np.pi))
            else:
                self.joint_angles[3] = feedback_msg_1.position % (2*np.pi)
        # joint 5
        control_mode_msg_2 = CyberGear.ControlModeMsg()
        control_mode_msg_2.can_id   = 2
        control_mode_msg_2.torque   = 0.0
        control_mode_msg_2.position = 0.0
        control_mode_msg_2.velocity = 0.0
        control_mode_msg_2.Kp       = 0.0
        control_mode_msg_2.Ki       = 0.0
        feedback_msg_2 = self.cybergear_motor_controller.controlMode(control_mode_msg_2)
        if feedback_msg_2 is not None:
            if feedback_msg_2.position < 0:
                self.joint_angles[4] = -((-feedback_msg_2.position) % (2*np.pi))
            else:
                self.joint_angles[4] = feedback_msg_2.position % (2*np.pi)

        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.name = self.joint_names
        self.joint_state.position = self.joint_angles
        self.joint_state_pub.publish(self.joint_state)


if __name__ == '__main__':
    rospy.init_node('motor_joint_state_publisher')
    motor_joint_state_publisher = MotorJointStatePublisher()
    rospy.spin()
