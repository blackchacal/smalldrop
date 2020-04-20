cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Tau.msg
  TrackingError.msg
  JointPositions.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS 
 message_runtime 
 roscpp 
 std_msgs
)
