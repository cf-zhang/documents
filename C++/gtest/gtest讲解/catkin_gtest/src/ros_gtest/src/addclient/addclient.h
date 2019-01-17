#ifndef _ADD_CLIENT_
#define _ADD_CLIENT_
#include "ros_gtest/AddTwoInts.h"
#include <ros/ros.h>
namespace ros_gtest
{
class AddClient
{
    public:
        AddClient();
        virtual ~AddClient(){};
        bool sendRequest(const long int a, const long int b, long int *sum);
        unsigned int getCounterSuccess()const {return counter_success;}
        unsigned int getCounterFail()const {return counter_fail;}
    private:
        ros::NodeHandle n;
        ros::ServiceClient client;
        unsigned int counter_success;
        unsigned int counter_fail;
};
}


#endif
