cmake_minimum_required(VERSION 3.0.2)
project(dynus_obstacles_ros1)

## C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  nav_msgs
  gazebo_msgs
  visualization_msgs
  tf2
  tf2_ros
)

## Catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynus_obstacles_ros1
  CATKIN_DEPENDS roscpp rospy roslib std_msgs geometry_msgs nav_msgs gazebo_msgs visualization_msgs tf2 tf2_ros
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare C++ executables
add_executable(dynus_obstacles_node
  src/dynus_obstacles_node.cpp
)

add_executable(fake_odom_node
  src/fake_odom_node.cpp
)

## Link executables
target_link_libraries(dynus_obstacles_node
  ${catkin_LIBRARIES}
)

target_link_libraries(fake_odom_node
  ${catkin_LIBRARIES}
)

## Install
install(TARGETS dynus_obstacles_node fake_odom_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
