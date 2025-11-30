# robot_navigation

This package provides navigation capabilities for the mobile robot including localization, path planning, and obstacle avoidance.

## Overview

The `robot_navigation` package configures the ROS navigation stack for autonomous mobile robot navigation. It includes AMCL for localization, Move Base for path planning, and costmap configurations for obstacle avoidance.

## Package Contents

### Launch Files (`launch/`)

#### [navigation.launch](file:///home/isma/Desktop/ros1-test/src/app/robot_navigation/launch/navigation.launch)

Complete navigation stack with localization and path planning.

**Usage:**
```bash
roslaunch robot_navigation navigation.launch
```

**Arguments:**
- `map_file` - Path to map YAML file (default: maps/map.yaml)
- `open_rviz` - Launch RViz for visualization (default: true)
- `move_forward_only` - Restrict robot to forward motion (default: false)

**Nodes Launched:**
- **map_server** - Serves the pre-built map
- **amcl** - Adaptive Monte Carlo Localization for robot localization
- **move_base** - Global and local path planning
- **rviz** - Visualization (optional)

### Configuration Files (`config/`)

#### Costmap Configuration

- **[costmap_common_params.yaml](file:///home/isma/Desktop/ros1-test/src/app/robot_navigation/config/costmap_common_params.yaml)** - Common costmap parameters (footprint, inflation, etc.)
- **[global_costmap_params.yaml](file:///home/isma/Desktop/ros1-test/src/app/robot_navigation/config/global_costmap_params.yaml)** - Global costmap configuration
- **[local_costmap_params.yaml](file:///home/isma/Desktop/ros1-test/src/app/robot_navigation/config/local_costmap_params.yaml)** - Local costmap configuration

#### Planner Configuration

- **[move_base_params.yaml](file:///home/isma/Desktop/ros1-test/src/app/robot_navigation/config/move_base_params.yaml)** - Move Base parameters
- **[dwa_local_planner_params.yaml](file:///home/isma/Desktop/ros1-test/src/app/robot_navigation/config/dwa_local_planner_params.yaml)** - DWA local planner parameters

### Maps (`maps/`)

Pre-built maps for navigation testing.

### RViz Configuration (`rviz/`)

- **navigation.rviz** - RViz configuration for navigation visualization

## Usage Examples

### Basic Navigation

```bash
# Launch navigation stack with default map
roslaunch robot_navigation navigation.launch

# Use custom map
roslaunch robot_navigation navigation.launch map_file:=/path/to/map.yaml

# Disable RViz
roslaunch robot_navigation navigation.launch open_rviz:=false
```

### Send Navigation Goals

Using RViz:
1. Launch navigation stack
2. Click "2D Pose Estimate" to set initial pose
3. Click "2D Nav Goal" to send navigation goal

Using command line:
```bash
# Send goal via rostopic
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position: {x: 2.0, y: 1.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

### Monitor Navigation Status

```bash
# Check move_base status
rostopic echo /move_base/status

# View current goal
rostopic echo /move_base/current_goal

# Monitor robot pose
rostopic echo /amcl_pose
```

## Key Topics

### Subscribed Topics
- `/scan` - Laser scan data for obstacle detection
- `/odom` - Odometry for localization
- `/map` - Static map from map_server

### Published Topics
- `/cmd_vel` - Velocity commands to robot
- `/move_base/global_costmap/costmap` - Global costmap
- `/move_base/local_costmap/costmap` - Local costmap
- `/amcl_pose` - Estimated robot pose

## Configuration Notes

### AMCL Parameters

The package uses AMCL with the following key settings:
- Particle filter: 500-3000 particles
- Laser model: likelihood_field
- Odometry model: differential drive

### DWA Local Planner

The DWA (Dynamic Window Approach) planner is configured for:
- Dynamic obstacle avoidance
- Smooth trajectory generation
- Velocity constraints based on robot capabilities

## Dependencies

- `map_server`
- `amcl`
- `move_base`
- `dwa_local_planner`
- `rviz`
- `tf`

## Related Packages

- **robot_description** - Robot URDF and visualization
- **robot_simulation** - Integrated simulation scenarios
- **robot_behavior** - High-level navigation behaviors
