cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_teleoperation)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  spacenav_node
  smalldrop_state
  roslib
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES smalldrop_teleoperation
 CATKIN_DEPENDS roscpp spacenav_node smalldrop_state
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)
link_directories(${EIGEN_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/remote_controller_mode.cpp
  src/${PROJECT_NAME}/remote_controller.cpp
  src/${PROJECT_NAME}/spacemouse.cpp
  src/${PROJECT_NAME}/teleoperation_actions.cpp
)

add_executable(default_teleoperation src/nodes/default_teleoperation.cpp)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(default_teleoperation ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############

catkin_add_gtest(t_remote_controller_mode_gtest test/smalldrop_teleoperation/t_remote_controller_mode_gtest.cpp)
catkin_add_gtest(t_spacemouse_gtest test/smalldrop_teleoperation/t_spacemouse_gtest.cpp)

if (TARGET t_remote_controller_mode_gtest)
target_link_libraries(t_remote_controller_mode_gtest ${PROJECT_NAME})
endif()
if (TARGET t_spacemouse_gtest)
target_link_libraries(t_spacemouse_gtest ${PROJECT_NAME})
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(t_spacemouse_rostest test/t_spacemouse.test test/smalldrop_teleoperation/t_spacemouse_rostest.cpp)
  target_link_libraries(t_spacemouse_rostest ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()