cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_vision)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  realsense2_camera
  realsense2_description
  smalldrop_segmentation
)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)
find_package(OpenCV 4 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES smalldrop_vision
 CATKIN_DEPENDS 
  roscpp 
  geometry_msgs 
  sensor_msgs
  realsense2_camera 
  realsense2_description
  smalldrop_segmentation
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/camera_d415.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(detect_wound src/nodes/detect_wound.cpp)

add_dependencies(detect_wound ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(detect_wound ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############

catkin_add_gtest(t_camera_d415_gtest test/smalldrop_vision/t_camera_d415_gtest.cpp)
if(TARGET t_camera_d415_gtest)
  target_link_libraries(t_camera_d415_gtest ${PROJECT_NAME})
endif()
