import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from functools import partial

import sys
import os
cur_dir = os.path.dirname(os.path.abspath(__file__))
CyberGear_dir = os.path.abspath(os.path.join(cur_dir, '..', 'motor_tools', 'CyberGear'))
if CyberGear_dir not in sys.path:
    sys.path.append(CyberGear_dir)
import CyberGear


class Traj_executor:
    def __init__(self):
        self.counter = 0
        self.traj_subscriber = rospy.Subscriber("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, self.traj_goal_callback)

        self.CyberGear_init()
        self.timer = rospy.Timer(rospy.Duration(self.time_from_start[0]), partial(self.execute, 1), oneshot=True)

    def CyberGear_init(self):
        self.cybergear_motor_ctrl = CyberGear.MotorCtrl('/dev/ttyUSB1', 921600, timeout=1)
        # joint 4
        enable_msg_1 = CyberGear.EnableMsg()
        enable_msg_1.can_id  = 1
        enable_msg_1.host_id = 253
        self.cybergear_motor_ctrl.enable(enable_msg_1)
        # joint 5
        enable_msg_2 = CyberGear.EnableMsg()
        enable_msg_2.can_id  = 2
        enable_msg_2.host_id = 253
        self.cybergear_motor_ctrl.enable(enable_msg_2)

    def traj_goal_callback(self, msg):
        self.goal_id = msg.goal_id
        self.joint_names = msg.goal.trajectory.joint_names
        self.points = msg.goal.trajectory.points
        rospy.loginfo(self.goal_id)
        self.positions = []
        self.velocities = []
        self.accelerations = []
        self.time_from_start = []
        for point in self.points:
            self.positions.append(point.positions)
            self.velocities.append(point.velocities)
            self.accelerations.append(point.accelerations)
            self.time_from_start.append(point.time_from_start.to_sec())
        
        # self.positions[i] is a list of positions of each joint at the ith time point
        # self.velocities[i] is a list of velocities of each joint at the ith time point
        # self.accelerations[i] is a list of accelerations of each joint at the ith time point
        # self.time_from_start[i] is the time from the start to the ith time point
        rospy.loginfo(self.positions)
        rospy.loginfo(self.velocities)
        rospy.loginfo(self.accelerations)
        rospy.loginfo(self.time_from_start)

    def execute(self, counter):
        if counter == len(self.time_from_start):
            return
        period = self.time_from_start[counter] - self.time_from_start[counter-1]
        self.timer = rospy.Timer(rospy.Duration(period), partial(self.execute, counter+1), oneshot=True)

        # joint 4
        control_mode_msg_1 = CyberGear.ControlModeMsg()
        control_mode_msg_1.can_id   = 1
        control_mode_msg_1.torque   = 0.0
        control_mode_msg_1.position = self.positions[counter][3]
        control_mode_msg_1.velocity = self.velocities[counter][3]
        control_mode_msg_1.Kp       = 1.0
        control_mode_msg_1.Ki       = 0.1
        feedback_msg_1 = self.cybergear_motor_ctrl.controlMode(control_mode_msg_1)
        if feedback_msg_1 is not None:
            print(feedback_msg_1.position)

        # joint 5
        control_mode_msg_2 = CyberGear.ControlModeMsg()
        control_mode_msg_2.can_id   = 2
        control_mode_msg_2.torque   = 0.0
        control_mode_msg_2.position = self.positions[counter][4]
        control_mode_msg_2.velocity = self.velocities[counter][4]
        control_mode_msg_2.Kp       = 1.0
        control_mode_msg_2.Ki       = 0.1
        feedback_msg_2 = self.cybergear_motor_ctrl.controlMode(control_mode_msg_2)
        if feedback_msg_2 is not None:
            print(feedback_msg_2.position)
        
if __name__ == '__main__':
    rospy.init_node('number_counter')
    Traj_executor()
    rospy.spin()