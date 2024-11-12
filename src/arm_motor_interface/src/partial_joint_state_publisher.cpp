#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

double joint_angles[3];

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("unitree_joint_states", 1);

  ros::Rate loop_rate(10);
  double counter = 0;

  while (ros::ok())
  {
    sensor_msgs::JointState msg;

    counter += 0.01;
    if (counter > 3.14)
    {
      counter = -3.14;
    }

    msg.header.stamp = ros::Time::now();
    msg.name = {"motor1", "motor2", "motor3"};


    // ************************* Assign joint angles here *******************
    joint_angles[0] = counter;
    joint_angles[1] = counter;
    joint_angles[2] = counter;

    msg.position = {joint_angles[0], joint_angles[1], joint_angles[2]};

    joint_state_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}