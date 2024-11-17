#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "serialPort/SerialPort.h"

double joint_angles[3];

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("unitree_joint_states", 1);
  ros::Rate loop_rate(10);

  SerialPort serial("/dev/ttyUSB1");

  while (ros::ok())
  {
    // joint 1
    MotorCmd cmd_1;
    MotorData data_1;
    cmd_1.motorType = MotorType::GO_M8010_6;
    cmd_1.id = 0;
    cmd_1.mode = 1;
    cmd_1.T = 0.0;
    cmd_1.W = 0.0;
    cmd_1.Pos = 0.0;
    cmd_1.K_P = 0.0;
    cmd_1.K_W = 0.0;
    serial.sendRecv(&cmd_1, &data_1);
    if (data_1.correct)
    {
      if (data_1.Pos < 0)
        joint_angles[0] = -((-data_1.Pos) % 6.28);
      else
        joint_angles[0] = data_1.Pos % 6.28;
    }
    // joint 2
    MotorCmd cmd_2;
    MotorData data_2;
    cmd_2.motorType = MotorType::GO_M8010_6;
    cmd_2.id = 1;
    cmd_2.mode = 1;
    cmd_2.T = 0.0;
    cmd_2.W = 0.0;
    cmd_2.Pos = 0.0;
    cmd_2.K_P = 0.0;
    cmd_2.K_W = 0.0;
    serial.sendRecv(&cmd_2, &data_2);
    if (data_2.correct)
    {
      if (data_2.Pos < 0)
        joint_angles[1] = -((-data_2.Pos) % 6.28);
      else
        joint_angles[1] = data_2.Pos % 6.28;
    }
    // joint 3
    MotorCmd cmd_3;
    MotorData data_3;
    cmd_3.motorType = MotorType::GO_M8010_6;
    cmd_3.id = 2;
    cmd_3.mode = 1;
    cmd_3.T = 0.0;
    cmd_3.W = 0.0;
    cmd_3.Pos = 0.0;
    cmd_3.K_P = 0.0;
    cmd_3.K_W = 0.0;
    serial.sendRecv(&cmd_3, &data_3);
    if (data_3.correct)
    {
      if (data_3.Pos < 0)
        joint_angles[2] = -((-data_3.Pos) % 6.28);
      else
        joint_angles[2] = data_3.Pos % 6.28;
    }

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name = {"motor1", "motor2", "motor3"};
    msg.position = {joint_angles[0], joint_angles[1], joint_angles[2]};
    joint_state_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}