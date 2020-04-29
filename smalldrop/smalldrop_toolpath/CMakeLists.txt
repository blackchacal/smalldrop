cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_toolpath)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  trajectory_msgs
  smalldrop_state
  smalldrop_segmentation
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)
find_package(OpenCV 4 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES toolpath_planner path_planner trajectory_planner joint_trajectory_planner
 CATKIN_DEPENDS geometry_msgs roscpp trajectory_msgs smalldrop_state smalldrop_segmentation cv_bridge
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
link_directories(${EIGEN_LIBRARY_DIRS})

add_library(toolpath_planner
  src/${PROJECT_NAME}/path.cpp
  src/${PROJECT_NAME}/toolpath.cpp
  src/${PROJECT_NAME}/zigzag.cpp
  src/${PROJECT_NAME}/parallel_lines.cpp
  src/${PROJECT_NAME}/grid.cpp
)
add_library(path_planner
  src/${PROJECT_NAME}/path.cpp
  src/${PROJECT_NAME}/line.cpp
  src/${PROJECT_NAME}/circle.cpp
  src/${PROJECT_NAME}/circular_spiral.cpp
)
add_library(trajectory_planner
  src/${PROJECT_NAME}/trajectory.cpp
  src/${PROJECT_NAME}/trajectory_planner.cpp
)
add_library(joint_trajectory_planner
  src/${PROJECT_NAME}/joint_trajectory_planner.cpp
)

add_dependencies(path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(toolpath_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(joint_trajectory_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_planner ${catkin_LIBRARIES})
target_link_libraries(toolpath_planner path_planner ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(trajectory_planner path_planner ${catkin_LIBRARIES})
target_link_libraries(joint_trajectory_planner ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############

catkin_add_gtest(t_paths_gtest test/smalldrop_toolpath/t_paths_gtest.cpp)
if(TARGET t_paths_gtest)
  target_link_libraries(t_paths_gtest path_planner)
endif()

catkin_add_gtest(t_trajectory_gtest test/smalldrop_toolpath/t_trajectory_gtest.cpp)
if(TARGET t_trajectory_gtest)
  target_link_libraries(t_trajectory_gtest path_planner trajectory_planner)
endif()

catkin_add_gtest(t_toolpaths_gtest test/smalldrop_toolpath/t_toolpaths_gtest.cpp)
if(TARGET t_toolpaths_gtest)
  target_link_libraries(t_toolpaths_gtest toolpath_planner)
endif()