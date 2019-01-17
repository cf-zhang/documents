#ifndef _ADD_SERVICE_
#define _ADD_SERVICE_
#include <ros/ros.h>
#include "ros_gtest/AddTwoInts.h"
namespace ros_gtest
{
class  AddServer
{
public:
    AddServer();
    virtual ~ AddServer(){};
    bool add(AddTwoInts::Request  &req,
         AddTwoInts::Response &res);
    unsigned int getCounter()const {return counter;}
private:
    unsigned int counter;
    ros::NodeHandle n;
    ros::ServiceServer service;
};

}

#endif