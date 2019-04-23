cmake_minimum_required(VERSION 2.8.3)
project(dijkstra)


add_executable(dijkstra greedy/dijkstra/dijkstra.cpp)
#target_link_libraries(actionlib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(actionlib actionlib_gencpp)
