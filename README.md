# Robot Project

This project contains a ROS 1 workspace for a mobile manipulator robot. It includes packages for simulation, arm configuration (MoveIt!), and navigation.

## Launch Scripts

The following launch scripts are available to run different aspects of the robot system:

### Mobile Manipulator Package
*   **`mobile_manipulator/launch/display.launch`**: Visualizes the robot model in Rviz. Useful for checking the URDF model.
*   **`mobile_manipulator/launch/simulation.launch`**: Starts the full simulation environment in Gazebo. It spawns the robot, starts state publishers, and launches a behavior tree node.

### Robot Arm Config Package (MoveIt!)
*   **`robot_arm_config/launch/demo.launch`**: A demo launch file for the MoveIt! configuration. It starts a fake joint state publisher and Rviz to visualize the arm planning without a real robot or full simulation.
*   **`robot_arm_config/launch/move_group.launch`**: Launches the main MoveIt! move group node, which handles motion planning, trajectory execution, and planning scene monitoring.
*   **`robot_arm_config/launch/planning_context.launch`**: Loads the URDF, SRDF, and other configuration files required for motion planning onto the parameter server.

### Robot Navigation Package
*   **`robot_navigation/launch/navigation.launch`**: Launches the complete navigation stack. This includes the Map Server, AMCL for localization, Move Base for path planning and control, and Rviz for visualization.
