import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np

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
        self.cybergear_motor_ctrl = CyberGear.MotorCtrl('/dev/ttyUSB1', 921600, timeout=1)
        enable_msg = CyberGear.EnableMsg()
        enable_msg.can_id  = 1
        enable_msg.host_id = 253
        self.cybergear_motor_ctrl.enable(enable_msg)
        enable_msg.can_id  = 2
        enable_msg.host_id = 253
        self.cybergear_motor_ctrl.enable(enable_msg)

    def unitree_joint_state_callback(self, msg):
        self.joint_angles[0] = msg.position[0]
        self.joint_angles[1] = msg.position[1]
        self.joint_angles[2] = msg.position[2]

    def timer_callback(self, event):
        control_mode_msg = CyberGear.ControlModeMsg()

        ######## Input the angle of motors (from -pi to pi) ########
        control_mode_msg.can_id   = 1
        control_mode_msg.torque   = 0.0
        control_mode_msg.position = 0.0
        control_mode_msg.velocity = 0.0
        control_mode_msg.Kp       = 0.0
        control_mode_msg.Ki       = 0.0
        feedback_msg = self.cybergear_motor_ctrl.controlMode(control_mode_msg)
        if feedback_msg is not None:
            self.joint_angles[3] = (feedback_msg.position + 4*np.pi) % (2*np.pi) - np.pi

        control_mode_msg.can_id   = 2
        control_mode_msg.torque   = 0.0
        control_mode_msg.position = 0.0
        control_mode_msg.velocity = 0.0
        control_mode_msg.Kp       = 0.0
        control_mode_msg.Ki       = 0.0
        feedback_msg = self.cybergear_motor_ctrl.controlMode(control_mode_msg)
        if feedback_msg is not None:
            self.joint_angles[4] = (feedback_msg.position + 4*np.pi) % (2*np.pi) - np.pi

        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.name = self.joint_names
        self.joint_state.position = self.joint_angles
        self.joint_state_pub.publish(self.joint_state)
        
if __name__ == '__main__':
    rospy.init_node('motor_joint_state_publisher')
    motor_joint_state_publisher = MotorJointStatePublisher()
    rospy.spin()
