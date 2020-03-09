cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_rviz)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS gazebo_ros geometry_msgs roscpp tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gazebo_models_to_rviz src/nodes/gazebo_models_to_rviz_node.cpp)

target_link_libraries(gazebo_models_to_rviz ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
