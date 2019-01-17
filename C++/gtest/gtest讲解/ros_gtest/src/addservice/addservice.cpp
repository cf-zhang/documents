
#include "addservice.h"
namespace ros_gtest
{
AddService::AddService()
{
    counter = 0;
    service = nh.advertiseService("add_two_ints", &AddService::add, this);
}

bool AddService::add(AddTwoInts::Request  &req,
         AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  counter++;
  return true;
}
}//namespace ros_gtest