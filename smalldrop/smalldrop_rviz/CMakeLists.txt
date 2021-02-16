cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_rviz)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  tf
  smalldrop_msgs
  smalldrop_state
  smalldrop_segmentation
  smalldrop_toolpath
  smalldrop_vision
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
  rospy
  tf 
  smalldrop_msgs 
  smalldrop_state 
  smalldrop_segmentation 
  smalldrop_toolpath
  smalldrop_vision
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
add_executable(toolpath_viewer_camera src/nodes/toolpath_viewer_camera.cpp)
add_executable(validate_wound_position src/nodes/validate_wound_position.cpp)

add_dependencies(toolpath_viewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(toolpath_viewer_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(validate_wound_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gazebo_models_to_rviz ${catkin_LIBRARIES})
target_link_libraries(toolpath_viewer toolpath_planner trajectory_planner ${catkin_LIBRARIES})
target_link_libraries(toolpath_viewer_camera toolpath_planner trajectory_planner ${catkin_LIBRARIES})
target_link_libraries(validate_wound_position toolpath_planner trajectory_planner ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
  PROGRAMS scripts/controller_error_plot.py
  PROGRAMS scripts/trajectory_tracking.py
  PROGRAMS scripts/debug_data.py
  # PROGRAMS scripts/segmentation_points_plot.py
  PROGRAMS scripts/trajectory_tracking_plot.sh
  PROGRAMS scripts/trajectory_tracking_errors_plot.sh
  # PROGRAMS scripts/show_joint_trajectory_profiles_plot.sh
  # PROGRAMS scripts/show_cartesian_trajectory_profiles_plot.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
