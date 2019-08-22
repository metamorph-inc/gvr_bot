cmake_minimum_required(VERSION 2.8.3)
project(gvr_bot_2dnav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  move_base
  gvr_bot_rplidar_hector_map_generated_launch_2019_5_10t16_55_19
  openmeta_testbench_simulation_executor
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gvr_bot_2dnav
#  CATKIN_DEPENDS move_base
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
