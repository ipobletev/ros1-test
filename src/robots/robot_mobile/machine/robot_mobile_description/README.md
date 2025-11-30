# robot_description

This package contains the robot's URDF description, visualization configurations, and display launch files.

## Overview

The `robot_description` package defines the complete robot model including the mobile base, manipulator arm, and sensors. It provides tools for visualizing the robot in RViz and loading the robot description for use in simulation and planning.

## Package Contents

### URDF Files (`urdf/`)

- **[robot.xacro](file:///home/isma/Desktop/ros1-test/src/app/robot_description/urdf/robot.xacro)** - Main robot description file
- **[mobile_base.xacro](file:///home/isma/Desktop/ros1-test/src/app/robot_description/urdf/mobile_base.xacro)** - Mobile base definition
- **[manipulator_arm.xacro](file:///home/isma/Desktop/ros1-test/src/app/robot_description/urdf/manipulator_arm.xacro)** - Manipulator arm definition
- **[sensors.xacro](file:///home/isma/Desktop/ros1-test/src/app/robot_description/urdf/sensors.xacro)** - Sensor definitions (lidar, cameras, etc.)

### Launch Files (`launch/`)

#### [display.launch](file:///home/isma/Desktop/ros1-test/src/app/robot_description/launch/display.launch)

Visualize the robot model in RViz without simulation.

**Usage:**
```bash
roslaunch robot_description display.launch
```

**Arguments:**
- `model` - Path to robot URDF/xacro file (default: robot.xacro)
- `gui` - Show joint state publisher GUI (default: true)
- `rvizconfig` - RViz configuration file (default: rviz/rviz.rviz)

**Features:**
- Loads robot description from xacro
- Launches joint state publisher (with GUI option)
- Starts robot state publisher for TF transforms
- Opens RViz with custom configuration

---

#### [gazebo.launch](file:///home/isma/Desktop/ros1-test/src/app/robot_description/launch/gazebo.launch)

Launch the robot in Gazebo simulation.

**Usage:**
```bash
roslaunch robot_description gazebo.launch
```

**Arguments:**
- `paused` - Start Gazebo paused (default: false)
- `gui` - Show Gazebo GUI (default: true)

**Features:**
- Starts Gazebo with empty world
- Spawns robot from URDF
- Launches robot state publisher
- Launches joint state publisher

### RViz Configuration (`rviz/`)

- **rviz.rviz** - Default RViz configuration for robot visualization

### Configuration (`config/`)

Contains robot-specific configuration files.

## Usage Examples

### Visualize Robot in RViz

```bash
# With joint state publisher GUI
roslaunch robot_mobile_description display.launch

# Without GUI
roslaunch robot_mobile_description display.launch gui:=false
```

### Simulate Robot in Gazebo

```bash
# Launch Gazebo simulation
roslaunch robot_mobile_description gazebo.launch

# Launch without GUI (headless)
roslaunch robot_mobile_description gazebo.launch gui:=false
```

### Load Robot Description Programmatically

```python
import rospy

# Robot description is loaded to parameter server
robot_description = rospy.get_param('/robot_description')
```

## Dependencies

- `urdf`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `rviz`
- `gazebo_ros`

## Related Packages

- **robot_simulation** - Comprehensive simulation scenarios
- **robot_arm** - MoveIt configuration for manipulation
- **robot_navigation** - Navigation stack configuration
