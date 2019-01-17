#ifndef ROS_LISTENER_H
#define ROS_LISTENER_H
#include <ros/ros.h>
#include "std_msgs/String.h"
namespace ros_gtest
{
class RosListener{
public:
    RosListener(){
    msg_sub = nh.subscribe("talker_msg", 1000, &RosListener::msgCallback, this);
    counter = 0;
    }
    unsigned int getCounter()const {return counter;}
    const std::string& getReceived() const{return received;}
    void msgCallback(const std_msgs::String & );
private:
    unsigned int counter;
    ros::NodeHandle nh;
    ros::Subscriber msg_sub;
    std::string received;
};
}
#endif
