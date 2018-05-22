cmake_minimum_required(VERSION 2.8.3)
project(sub_joystick_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(dual_stick_control src/dual_stick_control.cpp)

target_link_libraries(dual_stick_control ${catkin_LIBRARIES})