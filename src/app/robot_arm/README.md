# robot_arm

This package provides MoveIt motion planning configuration for the robot's manipulator arm.

## Overview

The `robot_arm` package contains the MoveIt configuration for controlling and planning motions for the robot's manipulator arm. It includes planning context, move group configuration, and demonstration launch files.

## Package Contents

### Launch Files (`launch/`)

#### [demo.launch](file:///home/isma/Desktop/ros1-test/src/app/robot_arm/launch/demo.launch)

Complete MoveIt demo with visualization.

**Usage:**
```bash
roslaunch robot_arm demo.launch
```

**Arguments:**
- `db` - Start database (default: false)
- `debug` - Enable debug mode (default: false)

**Features:**
- Loads robot description and SRDF
- Starts MoveIt move_group node
- Launches fake execution controllers
- Opens RViz with MoveIt plugin

---

#### [move_group.launch](file:///home/isma/Desktop/ros1-test/src/app/robot_arm/launch/move_group.launch)

Launch the MoveIt move_group node for motion planning.

**Usage:**
```bash
roslaunch robot_arm move_group.launch
```

**Arguments:**
- `allow_trajectory_execution` - Enable trajectory execution (default: true)
- `fake_execution` - Use fake controllers (default: true)
- `info` - Print extra information (default: true)
- `debug` - Enable debug mode (default: false)

---

#### [planning_context.launch](file:///home/isma/Desktop/ros1-test/src/app/robot_arm/launch/planning_context.launch)

Load MoveIt planning context (URDF, SRDF, kinematics, joint limits).

**Usage:**
```bash
roslaunch robot_arm planning_context.launch
```

**Arguments:**
- `load_robot_description` - Load robot description to parameter server (default: true)

### Configuration Files (`config/`)

MoveIt configuration files including:
- **SRDF** - Semantic Robot Description Format
- **Kinematics** - Kinematics solver configuration
- **Joint Limits** - Joint velocity and acceleration limits
- **Controllers** - Controller configuration
- **Planning** - Planning pipeline configuration

### RViz Configuration (`rviz/`)

- **rviz_moveit.rviz** - RViz configuration with MoveIt Motion Planning plugin

## Usage Examples

### Run MoveIt Demo

```bash
# Launch complete demo with RViz
roslaunch robot_arm demo.launch

# Launch in debug mode
roslaunch robot_arm demo.launch debug:=true
```

### Motion Planning with Python

```python
import rospy
import moveit_commander

# Initialize moveit_commander
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

# Instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()

# Instantiate a MoveGroupCommander object
group = moveit_commander.MoveGroupCommander("arm")

# Plan to a joint-space goal
joint_goal = group.get_current_joint_values()
joint_goal[0] = 0.5
joint_goal[1] = -0.5

group.go(joint_goal, wait=True)
group.stop()
```

### Motion Planning with C++

```cpp
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  
  moveit::planning_interface::MoveGroupInterface move_group("arm");
  
  // Plan to a pose goal
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.5;
  
  move_group.setPoseTarget(target_pose);
  move_group.move();
  
  return 0;
}
```

### Interactive Planning in RViz

1. Launch the demo: `roslaunch robot_arm demo.launch`
2. In RViz, use the Motion Planning plugin:
   - Drag the interactive marker to set goal pose
   - Click "Plan" to compute trajectory
   - Click "Execute" to run the motion

## Key Topics

### Subscribed Topics
- `/joint_states` - Current joint positions
- `/tf` - Transform tree

### Published Topics
- `/move_group/display_planned_path` - Planned trajectory visualization
- `/move_group/goal` - Motion planning goals
- `/execute_trajectory/goal` - Trajectory execution goals

### Action Servers
- `/move_group` - Main motion planning action server
- `/execute_trajectory` - Trajectory execution action server

## Planning Groups

The MoveIt configuration defines planning groups for different parts of the robot:
- **arm** - Manipulator arm group
- Additional groups may be defined in the SRDF

## Dependencies

- `moveit_core`
- `moveit_ros_planning`
- `moveit_ros_planning_interface`
- `moveit_ros_move_group`
- `moveit_planners_ompl`
- `moveit_ros_visualization`
- `robot_description`

## Related Packages

- **robot_description** - Robot URDF definition
- **robot_simulation** - Integrated simulation with MoveIt
- **robot_behavior** - High-level manipulation behaviors
