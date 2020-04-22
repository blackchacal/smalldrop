cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_bioprint)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  smalldrop_teleoperation
  smalldrop_toolpath
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES smalldrop_bioprint
 CATKIN_DEPENDS 
  geometry_msgs 
  roscpp 
  rospy 
  sensor_msgs 
  std_msgs 
  smalldrop_teleoperation 
  smalldrop_toolpath
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
  src/${PROJECT_NAME}/bioprinter.cpp
  src/${PROJECT_NAME}/system_config.cpp
)

add_executable(bioprinter src/nodes/bioprinter.cpp)

target_link_libraries(${PROJECT_NAME} yaml-cpp ${Boost_LIBRARIES} ${catkin_LIBRARIES})
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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(t_system_config_rostest test/t_system_config.test test/smalldrop_bioprint/t_system_config_rostest.cpp)
  target_link_libraries(t_system_config_rostest yaml-cpp ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

