#ifndef ROS_TALKER_H
#define ROS_TALKER_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
namespace ros_gtest
{
class RosTalker{
public:
    RosTalker(){
        msg_pub = nh.advertise<std_msgs::String>("talker_msg", 1000);
        counter = 0;
    }
    unsigned int getCounter()const {return counter;}
    ros::NodeHandle nh;
    ros::Publisher msg_pub;
    unsigned int counter;
    void talk(const std::string &str);
    int add(int, int);
};
}
#endif
