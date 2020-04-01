cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_bioprint)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES smalldrop_bioprint
 CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/bioprinter.cpp
  src/${PROJECT_NAME}/system_state.cpp
)

add_executable(bioprinter src/nodes/bioprinter.cpp)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(bioprinter ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############

catkin_add_gtest(t_bioprinter test/${PROJECT_NAME}/t_bioprinter.cpp)
if(TARGET t_bioprinter)
  target_link_libraries(t_bioprinter ${PROJECT_NAME})
endif()

