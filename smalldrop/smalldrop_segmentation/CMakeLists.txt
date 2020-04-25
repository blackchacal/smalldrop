cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_segmentation)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  std_msgs
)
find_package(OpenCV 4 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES smalldrop_segmentation
 CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/wound_segmentation_comanip.cpp
  src/${PROJECT_NAME}/wound_segmentation_comanip_convex_hull.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############

catkin_add_gtest(t_wound_segmentation_comanip_convex_hull_gtest test/smalldrop_segmentation/t_wound_segmentation_comanip_convex_hull_gtest.cpp)
if(TARGET t_wound_segmentation_comanip_convex_hull_gtest)
  target_link_libraries(t_wound_segmentation_comanip_convex_hull_gtest ${PROJECT_NAME})
endif()