#include "rostalker.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "rosTalker");
    ros_gtest::RosTalker rt;
    ros::Rate loop_rate(5);
    while(ros::ok()){
        rt.talk("hello world");
        loop_rate.sleep();
    }
    return 0;
}
