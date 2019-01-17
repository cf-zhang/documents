#include <gtest/gtest.h>
#include "./../src/talker/rostalker.h"
#include "./../src/listener/roslistener.h"

class TalkerListenerTest : public testing::Test
{
    public:
        virtual void SetUp();
        virtual void TearDown();

        ros_gtest::RosTalker *talker;
        ros_gtest::RosListener *listener;
};

void TalkerListenerTest::SetUp()
{
    talker = new ros_gtest::RosTalker();
    listener = new ros_gtest::RosListener();
}
void TalkerListenerTest::TearDown()
{
    delete talker;
    delete listener;
}


TEST_F(TalkerListenerTest, TalkerListenerTogether) 
{
    EXPECT_EQ(0u, talker->getCounter());
    EXPECT_EQ(0u, listener->getCounter());

    ros::Rate loop_rate(1);
    talker->talk("hello world");
    loop_rate.sleep();//保证能够发布完成，并接收成功
    EXPECT_EQ(1u, talker->getCounter());
    ASSERT_STREQ("hello world", listener->getReceived().c_str());
    EXPECT_EQ(1u, listener->getCounter());

    talker->talk("I love China!");
    loop_rate.sleep();
    EXPECT_EQ(2u, talker->getCounter());
    ASSERT_STREQ("I love China!", listener->getReceived().c_str());
    EXPECT_EQ(2u, listener->getCounter());

}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "rosTalkerAndListenerTest");
    ros::AsyncSpinner spinner(1);
    spinner.start();    
    return RUN_ALL_TESTS();
}
