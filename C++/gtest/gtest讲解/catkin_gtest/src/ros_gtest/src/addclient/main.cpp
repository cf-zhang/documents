#include "ros/ros.h"
#include "addclient.h"
#include <stdio.h>
void ss()




int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }  
  ros_gtest::AddClient addclient;

  long int a =  atoll(argv[1]);
  long int b =  atoll(argv[2]);
  long int sum;
  bool flag = false;
  flag = addclient.sendRequest(a, b, &sum);
  if(flag)
    ROS_INFO_STREAM(""<<a<<" + "<<b<<" = "<<sum);
  else
    ROS_ERROR_STREAM("calculate fail.");

  return 0;
}
