cmake_minimum_required(VERSION 2.8.3)
project(sub_trajectory)

find_package(catkin REQUIRED COMPONENTS message_generation rospy roscpp)
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib geometry_msgs dynamic_reconfigure)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY
   msg
   FILES
   ThrusterStatus.msg
   ThrusterCmd.msg
   StabilityMode.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   DIRECTORY
#   srv
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY
   action
   FILES
   GoToPose.action
   ExecuteWaypoint.action
 )

generate_dynamic_reconfigure_options(
  cfg/active_stability.cfg
  cfg/vector_thrust_control.cfg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   geometry_msgs
   std_msgs 
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sub_vision
  CATKIN_DEPENDS message_runtime 
#  DEPENDS system_lib
)
