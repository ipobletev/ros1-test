# robot_simulation

This package provides comprehensive launch files that integrate multiple robot subsystems for simulation and testing.

## Overview

The `robot_simulation` package consolidates launch scripts from multiple app packages (`robot_description`, `robot_navigation`, `robot_arm`, and `robot_behavior`) to provide easy-to-use simulation scenarios.

## Launch Files

### 1. full_simulation.launch

**Complete robot simulation with all subsystems**

Launches:
- Gazebo simulation environment
- Robot description and state publishers
- Navigation stack (AMCL + Move Base)
- MoveIt motion planning

**Usage:**
```bash
roslaunch robot_simulation full_simulation.launch
```

**Arguments:**
- `map_file` - Path to map YAML file (default: robot_navigation/maps/map.yaml)
- `open_rviz` - Open RViz visualization (default: true)
- `paused` - Start Gazebo paused (default: false)
- `gui` - Show Gazebo GUI (default: true)

---

### 2. gazebo_navigation.launch

**Gazebo simulation with navigation stack**

Launches:
- Gazebo simulation environment
- Robot description and state publishers
- Navigation stack (AMCL + Move Base)

**Usage:**
```bash
roslaunch robot_simulation gazebo_navigation.launch
```

**Arguments:**
- `map_file` - Path to map YAML file (default: robot_navigation/maps/map.yaml)
- `open_rviz` - Open RViz visualization (default: true)
- `paused` - Start Gazebo paused (default: false)
- `gui` - Show Gazebo GUI (default: true)
- `move_forward_only` - Restrict robot to forward motion only (default: false)

---

### 3. gazebo_moveit.launch

**Gazebo simulation with MoveIt motion planning**

Launches:
- Gazebo simulation environment
- Robot description and state publishers
- MoveIt motion planning

**Usage:**
```bash
roslaunch robot_simulation gazebo_moveit.launch
```

**Arguments:**
- `paused` - Start Gazebo paused (default: false)
- `gui` - Show Gazebo GUI (default: true)
- `debug` - Enable debug mode (default: false)
- `fake_execution` - Use fake execution controllers (default: false)

## Dependencies

This package depends on:
- `robot_description` - Robot URDF and visualization
- `robot_navigation` - Navigation configuration and maps
- `robot_arm` - MoveIt configuration
- `robot_behavior` - Robot behaviors
- Standard ROS packages: `gazebo_ros`, `rviz`, `move_base`, `amcl`, etc.

## Notes

- All launch files use `use_sim_time:=true` for Gazebo simulation
- The robot description is loaded from `robot_description/urdf/robot.xacro`
- Navigation uses the DWA local planner by default
- MoveIt can run with either real or fake execution controllers
