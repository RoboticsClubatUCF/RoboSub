cmake_minimum_required(VERSION 2.8.3)
project(sub_definition)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
