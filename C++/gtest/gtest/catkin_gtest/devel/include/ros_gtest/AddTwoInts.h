// Generated by gencpp from file ros_gtest/AddTwoInts.msg
// DO NOT EDIT!


#ifndef ROS_GTEST_MESSAGE_ADDTWOINTS_H
#define ROS_GTEST_MESSAGE_ADDTWOINTS_H

#include <ros/service_traits.h>


#include <ros_gtest/AddTwoIntsRequest.h>
#include <ros_gtest/AddTwoIntsResponse.h>


namespace ros_gtest
{

struct AddTwoInts
{

typedef AddTwoIntsRequest Request;
typedef AddTwoIntsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AddTwoInts
} // namespace ros_gtest


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ros_gtest::AddTwoInts > {
  static const char* value()
  {
    return "6a2e34150c00229791cc89ff309fff21";
  }

  static const char* value(const ::ros_gtest::AddTwoInts&) { return value(); }
};

template<>
struct DataType< ::ros_gtest::AddTwoInts > {
  static const char* value()
  {
    return "ros_gtest/AddTwoInts";
  }

  static const char* value(const ::ros_gtest::AddTwoInts&) { return value(); }
};


// service_traits::MD5Sum< ::ros_gtest::AddTwoIntsRequest> should match 
// service_traits::MD5Sum< ::ros_gtest::AddTwoInts > 
template<>
struct MD5Sum< ::ros_gtest::AddTwoIntsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ros_gtest::AddTwoInts >::value();
  }
  static const char* value(const ::ros_gtest::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_gtest::AddTwoIntsRequest> should match 
// service_traits::DataType< ::ros_gtest::AddTwoInts > 
template<>
struct DataType< ::ros_gtest::AddTwoIntsRequest>
{
  static const char* value()
  {
    return DataType< ::ros_gtest::AddTwoInts >::value();
  }
  static const char* value(const ::ros_gtest::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ros_gtest::AddTwoIntsResponse> should match 
// service_traits::MD5Sum< ::ros_gtest::AddTwoInts > 
template<>
struct MD5Sum< ::ros_gtest::AddTwoIntsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ros_gtest::AddTwoInts >::value();
  }
  static const char* value(const ::ros_gtest::AddTwoIntsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_gtest::AddTwoIntsResponse> should match 
// service_traits::DataType< ::ros_gtest::AddTwoInts > 
template<>
struct DataType< ::ros_gtest::AddTwoIntsResponse>
{
  static const char* value()
  {
    return DataType< ::ros_gtest::AddTwoInts >::value();
  }
  static const char* value(const ::ros_gtest::AddTwoIntsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROS_GTEST_MESSAGE_ADDTWOINTS_H