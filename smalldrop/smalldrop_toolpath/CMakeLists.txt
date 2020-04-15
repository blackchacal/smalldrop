cmake_minimum_required(VERSION 2.8.3)
project(smalldrop_toolpath)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  trajectory_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES toolpath_planner path_planner cartesian_space_trajectory_planner
 CATKIN_DEPENDS geometry_msgs roscpp trajectory_msgs
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
)
link_directories(${EIGEN_LIBRARY_DIRS})

add_library(path_planner
  src/${PROJECT_NAME}/path.cpp
  src/${PROJECT_NAME}/line.cpp
  src/${PROJECT_NAME}/circle.cpp
  src/${PROJECT_NAME}/circular_spiral.cpp
)

add_dependencies(path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/smalldrop_toolpath_node.cpp)

target_link_libraries(path_planner ${catkin_LIBRARIES})

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
