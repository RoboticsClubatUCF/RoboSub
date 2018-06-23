cmake_minimum_required(VERSION 2.8.3)
project(sub_thrusters_manager)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sub_thrusters_library
  sub_trajectory
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
#include_directories(../../../../jsoncpp/dist)

add_executable(sub_thruster_manager_node src/sub_thruster_manager_node.cpp)
target_link_libraries(sub_thruster_manager_node ${catkin_LIBRARIES} jsoncpp)

