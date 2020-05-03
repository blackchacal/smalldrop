cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_rviz)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  tf
  smalldrop_msgs
  smalldrop_state
  smalldrop_segmentation
  smalldrop_toolpath
  cv_bridge
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 
  gazebo_ros 
  geometry_msgs 
  roscpp 
  tf 
  smalldrop_msgs 
  smalldrop_state 
  smalldrop_segmentation 
  smalldrop_toolpath
  cv_bridge
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gazebo_models_to_rviz src/nodes/gazebo_models_to_rviz_node.cpp)
add_executable(toolpath_viewer src/nodes/toolpath_viewer.cpp)

add_dependencies(toolpath_viewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gazebo_models_to_rviz ${catkin_LIBRARIES})
target_link_libraries(toolpath_viewer toolpath_planner trajectory_planner ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
