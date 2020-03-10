cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_teleoperation)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES smalldrop_teleoperation
 CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/remote_controller_mode.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############

catkin_add_gtest(t_remote_controller_mode test/smalldrop_teleoperation/t_remote_controller_mode.cpp)

if (TARGET t_remote_controller_mode)
  target_link_libraries(t_remote_controller_mode ${PROJECT_NAME})
endif()