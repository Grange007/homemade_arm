import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult

class MoveItActionServer:
    def __init__(self) -> None:
        self.server = actionlib.ActionServer('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, goal_cb=self.on_goal, cancel_cb=self.on_cancel, auto_start=False)
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

    def on_goal(self, goal_handle):
        rospy.loginfo('Received goal')
        goal_handle.set_accepted()
        goal = goal_handle.get_goal()
        trajectory = goal.trajectory
        rospy.loginfo(trajectory)
        
        for point in trajectory.points:
            rospy.loginfo(point.positions)
        
        result = FollowJointTrajectoryResult()
        goal_handle.set_succeeded(result)
        
    def on_cancel(self, goal_handle):
        rospy.loginfo('Received cancel')
        result = FollowJointTrajectoryResult()
        goal_handle.set_canceled(result)

if __name__ == '__main__':
    rospy.init_node('moveit_action_server')
    moveit_action_server = MoveItActionServer()
    moveit_action_server.server.start()
    rospy.spin()