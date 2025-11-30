cmake_minimum_required(VERSION 3.0.2)
project(robot_app_behavior)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  move_base_msgs
  geometry_msgs
  std_msgs
)

# This macro looks for setup.py and installs the python module
catkin_python_setup()

catkin_package()

# Install the script
catkin_install_python(PROGRAMS
  scripts/mission_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
