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
    SerialPort serial = SerialPort("/dev/ttyUSB1");
    double zero_pos[3] = {0, 0, 0};

    public:
    Unitree_Traj_executor(ros::NodeHandle *nh) 
    {
        joint_state_pub = nh->advertise<sensor_msgs::JointState>("/unitree_joint_states", 1);    
        unitree_traj_sub = nh->subscribe("/arm_controller/follow_joint_trajectory/goal", 1, &Unitree_Traj_executor::traj_goal_callback, this);

        positions.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        velocities.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        accelerations.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        time_from_start.push_back(0);

        Untree_init();
        traj_executor = nh->createTimer(ros::Duration(0.1), std::bind(&Unitree_Traj_executor::timer_callback, this));
    }

    void Untree_init()
    {
        for (int motor_id = 0; motor_id < 3; motor_id++)
        {
            MotorCmd cmd;
            MotorData data;
            cmd.motorType = MotorType::GO_M8010_6;
            cmd.id = motor_id;
            cmd.mode = 0;
            cmd.T = 0.0;
            cmd.W = 0.0;
            cmd.Pos = 0.0;
            cmd.K_P = 0.0;
            cmd.K_W = 0.0;
            serial.sendRecv(&cmd, &data);
            if (data.correct)
                zero_pos[motor_id] = data.Pos;
            else
                motor_id--;
        }
        ros::Duration(0.5).sleep();
    }

    void traj_goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr &msg) 
    {
        double now = ros::Time::now().toSec();
        for (int i = 0; i < msg->goal.trajectory.points.size(); i++)
        {
            positions.push_back(msg->goal.trajectory.points[i].positions);
            velocities.push_back(msg->goal.trajectory.points[i].velocities);
            accelerations.push_back(msg->goal.trajectory.points[i].accelerations);
            time_from_start.push_back(msg->goal.trajectory.points[i].time_from_start.toSec() + now);
        }
    }

    void timer_callback()
    {
        double now = ros::Time::now().toSec();
        int counter = 0;
        for (int i = 0; i < time_from_start.size(); i++)
        {
            if (now < time_from_start[i])
                break;
            counter = i;
        }

        for (int motor_id = 0; motor_id < 3; motor_id++)
        {
            MotorCmd cmd;
            MotorData data;
            cmd.motorType = MotorType::GO_M8010_6;
            cmd.id = motor_id;
            cmd.mode = 1;
            cmd.T = 0.0;
            cmd.W = velocities[counter][motor_id] * 6.33;
            cmd.Pos = positions[counter][motor_id] * 6.33 + zero_pos[motor_id];
            cmd.K_P = 0.1;
            cmd.K_W = 0.0;
            serial.sendRecv(&cmd, &data);
            if (data.correct)
            {
                if (data.Pos - zero_pos[motor_id] < 0)
                    joint_angles[motor_id] = -fmod(-(data.Pos-zero_pos[motor_id])/6.33, 6.28);
                else
                    joint_angles[motor_id] = fmod((data.Pos-zero_pos[motor_id])/6.33, 6.28);
            }
        }

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"motor1", "motor2", "motor3"};
        msg.position = {joint_angles[0], joint_angles[1], joint_angles[2]};
        joint_state_pub.publish(msg);
    }
};


int main (int argc, char **argv)
{
    ros::init(argc, argv, "number_counter");
    ros::NodeHandle nh;
    Unitree_Traj_executor nc = Unitree_Traj_executor(&nh);
    ros::spin();
}