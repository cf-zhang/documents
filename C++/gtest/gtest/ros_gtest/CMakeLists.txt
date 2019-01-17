cmake_minimum_required(VERSION 2.8.3)
project(ros_gtest)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_service_files(FILES
  AddTwoInts.srv
)
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

message("the include file dir: "${catkin_INCLUDE_DIRS})
include_directories(
  include ${catkin_INCLUDE_DIRS}

)
add_executable(rostalker
  src/talker/main.cpp src/talker/rostalker.cpp)
add_executable(roslistener
  src/listener/main.cpp src/listener/roslistener.cpp)

add_executable(addserrvice
  src/addservice/main.cpp src/addservice/addservice.cpp)

add_executable(addclient
  src/addclient/main.cpp src/addclient/addclient.cpp)
target_link_libraries(addclient
  ${catkin_LIBRARIES} pthread
)

target_link_libraries(addserrvice
  ${catkin_LIBRARIES} pthread
)

target_link_libraries(roslistener
  ${catkin_LIBRARIES} pthread
)
target_link_libraries(rostalker
  ${catkin_LIBRARIES} pthread
)

catkin_add_gtest(talkerlistener-test 
                 src/talker/rostalker.cpp
                 src/listener/roslistener.cpp
                 test/test_talker_listener.cpp)
target_link_libraries(talkerlistener-test ${catkin_LIBRARIES})

catkin_add_gtest(addclientservice-test 
                  src/addservice/addservice.cpp
                  src/addclient/addclient.cpp
                  test/test_client_service.cpp)
target_link_libraries(addclientservice-test ${catkin_LIBRARIES})