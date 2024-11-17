#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
#include <vector>

#include "sensor_msgs/JointState.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "serialPort/SerialPort.h"


class Unitree_Traj_executor
{
    private:
    ros::Publisher joint_state_pub;
    ros::Subscriber unitree_traj_sub;
    ros::Timer traj_executor;
    double joint_angles[3] = {0, 0, 0}; // 6 joint angles
    std::vector<std::vector<double>> positions;
    std::vector<std::vector<double>> velocities;
    std::vector<std::vector<double>> accelerations;
    std::vector<double> time_from_start;
    SerialPort serial = SerialPort("/dev/ttyUSB0");

    public:
    Unitree_Traj_executor(ros::NodeHandle *nh) 
    {
        joint_state_pub = nh->advertise<sensor_msgs::JointState>("/unitree_joint_states", 1);    
        unitree_traj_sub = nh->subscribe("/arm_controller/follow_joint_trajectory/goal", 1, &Unitree_Traj_executor::traj_goal_callback, this);
        
        positions.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        velocities.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        accelerations.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        time_from_start.push_back(0);

        traj_executor = nh->createTimer(ros::Duration(0.1), std::bind(&Unitree_Traj_executor::timer_callback, this));
    }

    void timer_callback()
    {
        ros::Time now = ros::Time::now();
        int counter = 0;
        for (int i = 0; i < time_from_start.size(); i++)
        {
            if (now.toSec() < time_from_start[i])
                break;
            counter = i;
        }

        // joint 1
        MotorCmd cmd_1;
        MotorData data_1;
        cmd_1.motorType = MotorType::GO_M8010_6;
        cmd_1.id = 0;
        cmd_1.mode = 1;
        cmd_1.T = 0.0;
        cmd_1.W = this->velocities[counter][0] * 6.33;
        cmd_1.Pos = this->positions[counter][0] * 6.33;
        cmd_1.K_P = 0.05;
        cmd_1.K_W = 0.0;
        this->serial.sendRecv(&cmd_1, &data_1);
        if (data_1.correct)
        {
            if (data_1.Pos < 0)
                joint_angles[0] = -fmod(-data_1.Pos, 6.28) / 6.33;
            else
                joint_angles[0] = fmod(data_1.Pos, 6.28) / 6.33;
        }
        // joint 2
        MotorCmd cmd_2;
        MotorData data_2;
        cmd_2.motorType = MotorType::GO_M8010_6;
        cmd_2.id = 1;
        cmd_2.mode = 1;
        cmd_2.T = 0.0;
        cmd_2.W = this->velocities[counter][1] * 6.33;
        cmd_2.Pos = this->positions[counter][1] * 6.33;
        cmd_2.K_P = 0.05;
        cmd_2.K_W = 0.0;
        this->serial.sendRecv(&cmd_2, &data_2);
        if (data_2.correct)
        {
            if (data_2.Pos < 0)
                joint_angles[1] = -fmod(-data_2.Pos, 6.28) / 6.33;
            else
                joint_angles[1] = fmod(data_2.Pos, 6.28) / 6.33;
        }
        // joint 3
        MotorCmd cmd_3;
        MotorData data_3;
        cmd_3.motorType = MotorType::GO_M8010_6;
        cmd_3.id = 2;
        cmd_3.mode = 1;
        cmd_3.T = 0.0;
        cmd_3.W = this->velocities[counter][2] * 6.33;
        cmd_3.Pos = this->positions[counter][2] * 6.33;
        cmd_3.K_P = 0.05;
        cmd_3.K_W = 0.0;
        this->serial.sendRecv(&cmd_3, &data_3);
        if (data_3.correct)
        {
            if (data_3.Pos < 0)
                joint_angles[2] = -fmod(-data_3.Pos, 6.28) / 6.33;
            else
                joint_angles[2] = fmod(data_3.Pos, 6.28) / 6.33;
        }

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"motor1", "motor2", "motor3"};
        msg.position = {joint_angles[0], joint_angles[1], joint_angles[2]};
        joint_state_pub.publish(msg);
    }

    void traj_goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr &msg) 
    {
        positions.clear();
        velocities.clear();
        accelerations.clear();
        time_from_start.clear();
        for (int i = 0; i < msg->goal.trajectory.points.size(); i++)
        {
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