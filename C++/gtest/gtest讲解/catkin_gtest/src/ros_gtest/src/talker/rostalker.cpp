#include "rostalker.h"
namespace ros_gtest
{
void RosTalker::talk(const std::string &str){
    std_msgs::String msg;
    msg.data = str;
    msg_pub.publish(msg);
    counter++;
}

int RosTalker::add(int a, int b){
    return a + b;
}
}//namespace ros_gtest
