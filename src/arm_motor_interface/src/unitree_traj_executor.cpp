#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

#include "sensor_msgs/JointState.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include <vector>


class Unitree_Traj_executor {
    private:
    ros::Publisher joint_state_pub;
    ros::Subscriber unitree_traj_sub;
    ros::Timer traj_executor;
    double joint_angles[6] = {0, 0, 0, 0, 0, 0}; // 6 joint angles
    std::vector<std::vector<double>> positions;
    std::vector<std::vector<double>> velocities;
    std::vector<std::vector<double>> accelerations;
    std::vector<double> time_from_start;

    public:
    Unitree_Traj_executor(ros::NodeHandle *nh) 
    {
        joint_state_pub = nh->advertise<sensor_msgs::JointState>("/unitree_joint_states", 1);    
        unitree_traj_sub = nh->subscribe("/arm_controller/follow_joint_trajectory/goal", 1, &Unitree_Traj_executor::traj_goal_callback, this);
        traj_executor = nh->createTimer(ros::Duration(0.1), std::bind(&Unitree_Traj_executor::timer_callback, this));
        positions.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        velocities.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        accelerations.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        time_from_start.push_back(0);

    }

    void timer_callback()
    {
        /* The Timer callback */
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"motor1", "motor2", "motor3"};
        for (int i = 0; i < 3; i++) {
            msg.position.push_back(joint_angles[i]);
        }
        joint_state_pub.publish(msg);
    }


    void traj_goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr &msg) 
    {
        positions.clear();
        velocities.clear();
        accelerations.clear();
        time_from_start.clear();
        for (int i = 0; i < msg->goal.trajectory.points.size(); i++) {
            positions.push_back(msg->goal.trajectory.points[i].positions);
            velocities.push_back(msg->goal.trajectory.points[i].velocities);
            accelerations.push_back(msg->goal.trajectory.points[i].accelerations);
            time_from_start.push_back(msg->goal.trajectory.points[i].time_from_start.toSec());
        }   
    }
};
int main (int argc, char **argv)
{
    ros::init(argc, argv, "number_counter");
    ros::NodeHandle nh;
    Unitree_Traj_executor nc = Unitree_Traj_executor(&nh);
    ros::spin();
}