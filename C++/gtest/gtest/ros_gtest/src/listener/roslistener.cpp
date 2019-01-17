#include "roslistener.h"
namespace ros_gtest
{
void RosListener::msgCallback(const std_msgs::String & msg){
    received = msg.data;
    counter++;
}
}