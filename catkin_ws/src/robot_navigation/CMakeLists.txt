cmake_minimum_required(VERSION 3.0.2)
project(robot_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  move_base
  amcl
  map_server
  dwa_local_planner
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_navigation
#  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs move_base
#  DEPENDS system_lib
)

install(DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
