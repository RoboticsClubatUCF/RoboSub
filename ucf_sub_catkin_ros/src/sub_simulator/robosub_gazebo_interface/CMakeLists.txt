cmake_minimum_required(VERSION 2.8.3)
project(robosub_gazebo_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  roscpp
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robosub_gazebo_interface
#  CATKIN_DEPENDS gazebo_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  "src"
)

add_executable(robosub_gazebo_interface src/robosub_gazebo_interface.cpp)
target_link_libraries(robosub_gazebo_interface ${catkin_LIBRARIES})
