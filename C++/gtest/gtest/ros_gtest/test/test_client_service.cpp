#include <gtest/gtest.h>
#include "./../src/addclient/addclient.h"
#include "./../src/addservice/addservice.h"



class ClientTest : public testing::Test
{
    public:
        virtual void SetUp(){};
        virtual void TearDown(){};
    

        ros_gtest::AddClient addclient;
};


class ClientServiceTest : public testing::Test
{
    public:
        virtual void SetUp();
        virtual void TearDown();

        ros::NodeHandle nh;
        ros_gtest::AddClient *addclient;
        ros_gtest::AddService *addservice;
};

void ClientServiceTest::SetUp()
{
    addservice = new ros_gtest::AddService();
    addclient = new ros_gtest::AddClient();

}
void ClientServiceTest::TearDown()
{
    delete addclient;
    delete addservice;
}


TEST_F(ClientTest, AddClientAlone) 
{
  long int sum;
  EXPECT_FALSE(addclient.sendRequest(1, 2, &sum));
  EXPECT_EQ(0u, addclient.getCountSuccess());
  EXPECT_EQ(1u, addclient.getCountFail());
}



TEST_F(ClientServiceTest, AddClientServiceTogether)
{
  long int sum = 0;
  EXPECT_TRUE(addclient->sendRequest(1, 2, &sum));
  EXPECT_EQ(3u, sum);
  EXPECT_EQ(1u, addservice->getCounter());
  EXPECT_EQ(1u, addclient->getCountSuccess());
  EXPECT_EQ(0u, addclient->getCountFail());

  EXPECT_TRUE(addclient->sendRequest(15, 2, &sum));
  EXPECT_EQ(17u, sum);
  EXPECT_EQ(2u, addservice->getCounter());
  EXPECT_EQ(2u, addclient->getCountSuccess());
  EXPECT_EQ(0u, addclient->getCountFail());

}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "rosClientServiceTest");
    ros::AsyncSpinner spinner(1);//同一个节点内包含客户端和服务端，推荐
    spinner.start();
    return RUN_ALL_TESTS();
}
