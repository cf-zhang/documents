#include "addclient.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros_gtest::AddClient addclient;

  long int sum = 0;
  long int a = atol(argv[1]);
  long int b = atol(argv[2]);
  bool ret = false;
  ret = addclient.sendRequest(a, b, &sum);
  if(ret)
    ROS_INFO_STREAM("result: "<<sum);
  else
    ROS_ERROR_STREAM("fail to calculate "<<a <<" + "<<b);
  return 0;
}