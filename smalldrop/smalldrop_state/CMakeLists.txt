cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_state)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  smalldrop_msgs
  controller_manager_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs smalldrop_msgs controller_manager_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/system_state.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
