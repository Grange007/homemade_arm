import rospy

from control_msgs.msg import FollowJointTrajectoryActionGoal
from sensor_msgs.msg import JointState

import sys
import os

cur_dir = os.path.dirname(os.path.abspath(__file__))
Unitree_dir = os.path.abspath(os.path.join(cur_dir, '..', 'motor_tools', 'Unitree'))
if Unitree_dir not in sys.path:
    sys.path.append(Unitree_dir)
import Unitree
Cybergear_dir = os.path.abspath(os.path.join(cur_dir, '..', 'motor_tools', 'Cybergear'))
if Cybergear_dir not in sys.path:
    sys.path.append(Cybergear_dir)
import Cybergear


class Traj_executor:
    def __init__(self):
        self.traj_subscriber = rospy.Subscriber("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, self.traj_goal_callback)
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_state = JointState()

        self.positions = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self.velocities = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self.accelerations = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self.time_from_start = [0]

        self.zero_positions = {}
        self.Unitree_zero = False
        self.Cybergear_zero = False
        self.Kp = [3.0, 3.0, 3.0, 5.0, 5.0, 5.0]
        self.Kv = [0.2, 0.2, 0.2, 4.0, 4.0, 4.0]

        self.Unitree_init('/dev/ttyUSB0', 4000000)
        self.Cybergear_init('/dev/ttyUSB1', 921600)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
    
    def Unitree_init(self, port, baudrate):
        self.Unitree_controller = Unitree.MotorController(port, baudrate, 1)
        for i in range(0, 3):
            while True:
                print("Unitree", i)
                control_msg = Unitree.ControlMsg()
                control_msg.id       = i
                control_msg.status   = 0
                control_msg.torque   = 0.0
                control_msg.position = 0.0
                control_msg.velocity = 0.0
                control_msg.Kp       = 0.0
                control_msg.Kv       = 0.0
                feedback_msg = self.Unitree_controller.control(control_msg)
                if feedback_msg != None:
                    self.zero_positions[i] = feedback_msg.position
                    break

    def Cybergear_init(self, port, baudrate):
        self.Cybergear_controller = Cybergear.MotorController(port, baudrate, 1)
        for i in range(0, 2):
            while True:
                print("Cybergear", i + 1)
                enable_msg = Cybergear.EnableMsg()
                enable_msg.can_id  = i + 1
                enable_msg.host_id = 253
                feedback_msg = self.Cybergear_controller.enable(enable_msg)
                if feedback_msg != None:
                    self.zero_positions[i + 3] = feedback_msg.position
                    break

    def traj_goal_callback(self, msg):
        now = rospy.Time.now().to_sec()
        self.goal_id = msg.goal_id
        self.joint_names = msg.goal.trajectory.joint_names
        self.points = msg.goal.trajectory.points
        rospy.loginfo(self.goal_id)
        
        # self.positions[i] is a list of positions of each joint at the ith time point
        # self.velocities[i] is a list of velocities of each joint at the ith time point
        # self.accelerations[i] is a list of accelerations of each joint at the ith time point
        # self.time_from_start[i] is the time from the start to the ith time point
        for point in self.points:
            self.positions.append(point.positions)
            self.velocities.append(point.velocities)
            self.accelerations.append(point.accelerations)
            self.time_from_start.append(point.time_from_start.to_sec() + now)
        
        rospy.loginfo(self.positions)
        rospy.loginfo(self.velocities)
        rospy.loginfo(self.accelerations)
        rospy.loginfo(self.time_from_start)

    def timer_callback(self, event):
        now = rospy.Time.now().to_sec()
        for i in range(len(self.time_from_start)):
            if now < self.time_from_start[i]:
                break
            counter = i

        for i in range(0, 3):
            control_msg = Unitree.ControlMsg()
            control_msg.id       = i
            control_msg.status   = 1
            control_msg.torque   = 0.0
            control_msg.position = self.positions[counter][i] * 6.33 + self.zero_positions[i]
            control_msg.velocity = self.velocities[counter][i] * 6.33
            if self.Unitree_zero:
                control_msg.Kp = 0.0
                control_msg.Kv = 0.0
            else:
                control_msg.Kp = self.Kp[i]
                control_msg.Kv = self.Kv[i]
            feedback_msg = self.Unitree_controller.control(control_msg)
            if feedback_msg != None:
                if feedback_msg.position - self.zero_positions[i] < 0:
                    self.joint_angles[i] = -(((-feedback_msg.position + self.zero_positions[i]) / 6.33) % 6.28)
                else:
                    self.joint_angles[i] = ((feedback_msg.position - self.zero_positions[i]) / 6.33) % 6.28
        
        for i in range(0, 2):
            control_mode_msg = Cybergear.ControlModeMsg()
            control_mode_msg.can_id   = i + 1
            control_mode_msg.torque   = 0.0
            control_mode_msg.position = self.positions[counter][i + 3] + self.zero_positions[i + 3]
            control_mode_msg.velocity = self.velocities[counter][i + 3]
            if self.Cybergear_zero:
                control_mode_msg.Kp = 0.0
                control_mode_msg.Kv = 0.0
            else:
                control_mode_msg.Kp = self.Kp[i + 3]
                control_mode_msg.Kv = self.Kv[i + 3]
            feedback_msg = self.Cybergear_controller.controlMode(control_mode_msg)
            if feedback_msg != None:
                if feedback_msg.position - self.zero_positions[i + 3] < 0:
                    self.joint_angles[i + 3] = -((-feedback_msg.position + self.zero_positions[i + 3]) % 6.28)
                else:
                    self.joint_angles[i + 3] = (feedback_msg.position - self.zero_positions[i + 3]) % 6.28

        position = (self.positions[counter][5] / 3.14 * 180) / 270 * 2000 + 500
        self.end_gear.send_data(position, 0.1)
        received_position = self.end_gear.get_position()
        self.joint_angles[5] = (received_position - 500) / 2000 * 270 / 180 * 3.14
        
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.name = self.joint_names
        self.joint_state.position = self.joint_angles
        self.joint_state_pub.publish(self.joint_state)
        # print(now, self.joint_state.position)


if __name__ == '__main__':
    rospy.init_node('trajectory_executor')
    Traj_executor()
    rospy.spin()