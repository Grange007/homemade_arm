import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np

import sys
import os

class MotorJointStatePublisher:
    def __init__(self) -> None:
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.unitree_joint_state_sub = rospy.Subscriber('unitree_joint_states', JointState, self.unitree_joint_state_callback)
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.joint_state = JointState()

        self.test_counter = 0

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def unitree_joint_state_callback(self, msg):
        self.joint_angles[0] = msg.position[0]
        self.joint_angles[1] = msg.position[1]
        self.joint_angles[2] = msg.position[2]


    def timer_callback(self, event):
        self.test_counter += 0.01
        if self.test_counter > 3.14:
            self.test_counter = -3.14
        self.joint_angles = [self.test_counter, self.test_counter, self.test_counter, self.test_counter, self.test_counter]

        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.name = self.joint_names
        self.joint_state.position = self.joint_angles
        self.joint_state_pub.publish(self.joint_state)
        
if __name__ == '__main__':
    rospy.init_node('motor_joint_state_publisher')
    motor_joint_state_publisher = MotorJointStatePublisher()
    rospy.spin()
