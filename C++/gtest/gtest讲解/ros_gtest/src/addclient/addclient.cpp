#include "addclient.h"
#include "ros_gtest/AddTwoInts.h"
namespace ros_gtest
{
AddClient::AddClient()
{
    client = nh.serviceClient<AddTwoInts>("add_two_ints");
    counter_success = 0;
    counter_fail = 0;
}


bool AddClient::sendRequest(const long int a, 
                             const long int b,
                             long int *sum)
{
    AddTwoInts srv;
    srv.request.a = a;
    srv.request.b = b;
    
    if(client.call(srv))
    {
        *sum = srv.response.sum;
        counter_success++;
        return true;
    }
    counter_fail++;
    return false;
}

}//namespace ros_gtest
