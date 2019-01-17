#include "gtest/gtest.h"
#include "./../src/addclient/addclient.h"
#include "./../src/addservice/addservice.h"

class AddClientTest : public testing::Test {
 protected:
  virtual void SetUp() {
	  std::cout<<"in setUps"<<std::endl;

  }

  virtual void TearDown() {
	  std::cout<<"in tear down"<<std::endl;
   }
   ros_gtest::AddClient addclient;
};

TEST_F(AddClientTest, ClientTestTogether) {
  EXPECT_EQ(0u, addclient.getCounterSuccess());
  EXPECT_EQ(0u, addclient.getCounterFail());
  long int sum = 0;
  EXPECT_FALSE(addclient.sendRequest(1l, 2l, &sum));
  EXPECT_EQ(0, sum);
  EXPECT_EQ(1u, addclient.getCounterFail());
  EXPECT_EQ(0u, addclient.getCounterSuccess());

}






class AddServiceAndClientTest : public testing::Test {
 protected:
  virtual void SetUp() {
	std::cout<<"in setUps"<<std::endl;
    addclient = new ros_gtest::AddClient();
    addserver = new ros_gtest::AddServer();
  }

  virtual void TearDown() {
	std::cout<<"in tear down"<<std::endl;
    delete addclient;
    delete addserver;
   }
   ros_gtest::AddClient *addclient;
   ros_gtest::AddServer *addserver;
};


TEST_F(AddServiceAndClientTest, ServiceAndClientTestTogether) {
  EXPECT_EQ(0u, addclient->getCounterSuccess());
  EXPECT_EQ(0u, addclient->getCounterFail());
  long int sum = 0;
  EXPECT_TRUE(addclient->sendRequest(1l, 2l, &sum));
  EXPECT_EQ(3, sum);
  EXPECT_EQ(1u, addclient->getCounterSuccess());
  EXPECT_EQ(0u, addclient->getCounterFail());
  EXPECT_EQ(1u, addserver->getCounter());
}


 int main(int argc, char **argv){
   
   testing::InitGoogleTest(&argc, argv);
   ros::init(argc, argv, "add_client_and_server_node");
    ros::AsyncSpinner spinner(1);//同一个节点内包含客户端和服务端，推荐
    spinner.start();   
   return RUN_ALL_TESTS();
 }

