﻿#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include "serialPort/SerialPort.h"
#include <unistd.h>

double joint_angles[3];

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("unitree_joint_states", 1);

  ros::Rate loop_rate(10);
  // double counter = 0;

  SerialPort serial("/dev/ttyUSB0");
  MotorCmd cmd;
  MotorData data;

  while (ros::ok())
  {
    sensor_msgs::JointState msg;

    // counter += 0.01;
    // if (counter > 3.14)
    // {
    //   counter = -3.14;
    // }

    msg.header.stamp = ros::Time::now();
    msg.name = {"motor1", "motor2", "motor3"};


    // ************************* Assign joint angles here *******************
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id = 0;
    cmd.mode = 1;
    cmd.K_P = 0.0;
    cmd.K_W = 0.0;
    cmd.Pos = 0.0;
    cmd.W = 0.0;
    cmd.T = 0.0;
    serial.sendRecv(&cmd,&data);
    if (data.correct)
      joint_angles[0] = data.Pos;

    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id = 1;
    cmd.mode = 1;
    cmd.K_P = 0.0;
    cmd.K_W = 0.0;
    cmd.Pos = 0.0;
    cmd.W = 0.0;
    cmd.T = 0.0;
    serial.sendRecv(&cmd,&data);
    if (data.correct)
      joint_angles[1] = data.Pos;
    joint_angles[1] = counter;

    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id = 2;
    cmd.mode = 1;
    cmd.K_P = 0.0;
    cmd.K_W = 0.0;
    cmd.Pos = 0.0;
    cmd.W = 0.0;
    cmd.T = 0.0;
    serial.sendRecv(&cmd,&data);
    if (data.correct)
      joint_angles[2] = data.Pos;

    msg.position = {joint_angles[0], joint_angles[1], joint_angles[2]};

    joint_state_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}