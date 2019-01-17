#include "roslistener.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "rosListener");
    ros_gtest::RosListener rl;
    ros::spin();
    return 0;
}
