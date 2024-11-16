import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal

class Traj_executor:
    def __init__(self):
        self.counter = 0
        self.traj_subscriber = rospy.Subscriber("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, self.traj_goal_callback)
        
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
        
if __name__ == '__main__':
    rospy.init_node('number_counter')
    Traj_executor()
    rospy.spin()