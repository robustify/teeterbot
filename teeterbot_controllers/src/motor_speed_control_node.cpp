#include <ros/ros.h>
#include "MotorSpeedController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_speed_controller");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  teeterbot_controllers::MotorSpeedController node(n, pn);

  ros::spin();
}
