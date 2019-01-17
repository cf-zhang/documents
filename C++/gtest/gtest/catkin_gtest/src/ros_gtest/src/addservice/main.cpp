#include "ros/ros.h"

#include "addservice.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server"); 
  ros_gtest::AddServer addserver;
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}