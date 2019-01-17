
#include "addservice.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros_gtest::AddService addservice;
  ros::spin();
  return 0;
}