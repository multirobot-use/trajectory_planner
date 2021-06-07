cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} "cmake/Modules/" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( Eigen3 REQUIRED )
find_package(ACADO REQUIRED)

link_directories(${ACADO_LIBRARY_DIRS})


include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${ACADO_INCLUDE_DIRS} include )


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_srvs
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  trajectory_msgs
)

add_service_files(
  FILES
  WaypointSrv.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
)

# catkin_package(
#  INCLUDE_DIRS include
#   LIBRARIES mission_planner
#   CATKIN_DEPENDS roscpp rospy std_msgs std_srvs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mission_planner
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(mission_planner src/mission_planner.cpp include/mission_planner_types.hpp include/log.h)

add_executable(${PROJECT_NAME}_node test/main.cpp src/mission_planner_ros.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${ACADO_SHARED_LIBRARIES} mission_planner)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} )
