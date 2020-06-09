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
  image_transport
  cv_bridge
  camera_info_manager
  gazebo_ros
  pcl_ros
)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)
find_package(OpenCV 4 REQUIRED)
find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES smalldrop_vision realsense_gazebo
 CATKIN_DEPENDS 
  roscpp 
  geometry_msgs 
  sensor_msgs
  realsense2_camera 
  realsense2_description
  smalldrop_segmentation
  gazebo_ros
  pcl_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  include/gazebo_plugins
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/camera_d415.cpp
)

add_library(realsense_gazebo SHARED 
  src/gazebo_plugins/gazebo_ros_realsense.cpp
  src/gazebo_plugins/RealSensePlugin.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(realsense_gazebo ${catkin_EXPORTED_TARGETS})

add_executable(detect_wound src/nodes/detect_wound.cpp)

add_dependencies(detect_wound ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(detect_wound ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(realsense_gazebo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

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
