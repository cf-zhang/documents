#include "addservice.h"

namespace ros_gtest{

AddServer::AddServer()
{
   service = n.advertiseService("add_two_ints", &AddServer::add, this);
   counter = 0;
}

bool AddServer::add(AddTwoInts::Request  &req,
         AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;

  counter++;
  return true;
}
}