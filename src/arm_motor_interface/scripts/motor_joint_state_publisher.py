import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np

class MotorJointStatePublisher:
    def __init__(self) -> None:
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.joint_state = JointState()
        
        self.test_counter = 0;
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
    
    def timer_callback(self, event):
        self.test_counter += 0.01
        if self.test_counter > 3.14:
            self.test_counter = -3.14
        self.joint_angles = [self.test_counter, self.test_counter, self.test_counter, self.test_counter, self.test_counter]

        ######## Input the angle of motors (from -pi to pi) ##########
        # self.joint_angles[0] = 
        # self.joint_angles[1] = 
        # self.joint_angles[2] = 
        # self.joint_angles[3] = 
        # self.joint_angles[4] = 
         
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.name = self.joint_names
        self.joint_state.position = self.joint_angles
        self.joint_state_pub.publish(self.joint_state)
        
if __name__ == '__main__':
    rospy.init_node('motor_joint_state_publisher')
    motor_joint_state_publisher = MotorJointStatePublisher()
    rospy.spin()
        