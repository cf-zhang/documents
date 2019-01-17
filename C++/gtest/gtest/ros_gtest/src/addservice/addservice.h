#ifndef _ADD_SERVICE_
#define _ADD_SERVICE_
#include "ros/ros.h"
#include "ros_gtest/AddTwoInts.h"
namespace ros_gtest
{
    class AddService
    {
        public:
            AddService();
            ~AddService(){};
            unsigned int getCounter()const {return counter;}
        private:
            ros::NodeHandle nh;
            ros::ServiceServer service;
            unsigned int counter;
            bool add(AddTwoInts::Request  &req,
                    AddTwoInts::Response &res);
    };
}


#endif