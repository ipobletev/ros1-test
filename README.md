# Robot Project

This project contains a ROS 1 workspace for a mobile manipulator robot. It includes packages for simulation, arm configuration (MoveIt!), and navigation.

## Launch Commands

Before running any commands, ensure you have sourced the workspace:

```bash
source /opt/ros/noetic/setup.bash && source devel/setup.bash
```

### Mobile Manipulator Package

*   `roslaunch robot_description display.launch`: Visualizes the robot model in Rviz. Useful for checking the URDF model.
*   `roslaunch robot_description simulation.launch`: Starts the full simulation environment in Gazebo. It spawns the robot, starts state publishers, and launches a behavior tree node.

### Robot Arm Config Package (MoveIt!)

*   `roslaunch robot_arm demo.launch`: A demo launch file for the MoveIt! configuration. It starts a fake joint state publisher and Rviz to visualize the arm planning without a real robot or full simulation.
*   `roslaunch robot_arm move_group.launch`: Launches the main MoveIt! move group node, which handles motion planning, trajectory execution, and planning scene monitoring.
*   `roslaunch robot_arm planning_context.launch`: Loads the URDF, SRDF, and other configuration files required for motion planning onto the parameter server.

### Robot Navigation Package

*   `roslaunch robot_navigation navigation.launch`: Launches the complete navigation stack. This includes the Map Server, AMCL for localization, Move Base for path planning and control, and Rviz for visualization.
