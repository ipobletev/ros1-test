cmake_minimum_required(VERSION 3.0.2)
project(mobile_manipulator)

# Find the catkin build system and other required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf
  xacro
  gazebo_ros
  nav_msgs
)

# Define the python script installation (for the behavior tree)
catkin_install_python(PROGRAMS
  scripts/simple_behavior_tree.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Define the catkin package parameters
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobile_manipulator
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# Install directories (launch files, config, urdf, etc.)
install(DIRECTORY launch config urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
