# robot_behavior

This package provides high-level behavior and mission control for the robot, coordinating navigation and manipulation actions.

## Overview

The `robot_behavior` package implements behavior trees and action servers for complex robot missions. It coordinates navigation goals and manipulation tasks to accomplish high-level objectives.

## Package Contents

### Scripts (`scripts/`)

#### [mission_node.py](file:///home/isma/Desktop/ros1-test/src/app/robot_behavior/scripts/mission_node.py)

Main mission control node that orchestrates robot behaviors.

**Usage:**
```bash
rosrun robot_behavior mission_node.py
```

---

#### [nav_action.py](file:///home/isma/Desktop/ros1-test/src/app/robot_behavior/scripts/nav_action.py)

Navigation action server for executing navigation goals.

**Usage:**
```bash
rosrun robot_behavior nav_action.py
```

---

#### [arm_action.py](file:///home/isma/Desktop/ros1-test/src/app/robot_behavior/scripts/arm_action.py)

Manipulation action server for executing arm movements.

**Usage:**
```bash
rosrun robot_behavior arm_action.py
```

---

#### [main_mission.py](file:///home/isma/Desktop/ros1-test/src/app/robot_behavior/scripts/main_mission.py)

Main mission script for executing complete robot missions.

**Usage:**
```bash
rosrun robot_behavior main_mission.py
```

### Python Modules (`src/robot_behavior/`)

Reusable Python modules for behavior implementation.

## Usage Examples

### Run Mission Node

```bash
# Start the mission control node
rosrun robot_behavior mission_node.py
```

### Execute Navigation Action

```python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Create action client
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

# Create and send goal
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.pose.position.x = 2.0
goal.target_pose.pose.position.y = 1.0
goal.target_pose.pose.orientation.w = 1.0

client.send_goal(goal)
client.wait_for_result()
```

### Execute Manipulation Action

```python
import rospy
import moveit_commander

# Initialize MoveIt
moveit_commander.roscpp_initialize([])
rospy.init_node('manipulation_example')

# Get arm planning group
arm = moveit_commander.MoveGroupCommander("arm")

# Plan and execute to named target
arm.set_named_target("home")
arm.go(wait=True)
```

### Complete Mission Example

```python
#!/usr/bin/env python
import rospy
from robot_behavior import NavigationBehavior, ManipulationBehavior

def main():
    rospy.init_node('mission_example')
    
    # Navigate to target location
    nav = NavigationBehavior()
    nav.navigate_to(x=2.0, y=1.0)
    
    # Perform manipulation
    arm = ManipulationBehavior()
    arm.pick_object()
    
    # Navigate back
    nav.navigate_to(x=0.0, y=0.0)
    arm.place_object()

if __name__ == '__main__':
    main()
```

## Architecture

The package uses a behavior-based architecture:

1. **Mission Node** - High-level mission coordinator
2. **Action Servers** - Navigation and manipulation action servers
3. **Behavior Trees** - Structured behavior execution
4. **State Machine** - Mission state management

## Key Topics

### Subscribed Topics
- `/move_base/status` - Navigation status
- `/move_group/status` - Manipulation status
- `/joint_states` - Current joint positions
- `/amcl_pose` - Robot localization

### Published Topics
- `/move_base/goal` - Navigation goals
- `/move_group/goal` - Manipulation goals
- `/mission_status` - Current mission status

## Action Servers

- `/move_base` - Navigation action server
- `/move_group` - Manipulation action server
- `/mission_control` - High-level mission action server

## Dependencies

- `rospy`
- `actionlib`
- `move_base_msgs`
- `moveit_msgs`
- `geometry_msgs`
- `std_msgs`

## Related Packages

- **robot_navigation** - Navigation stack for mobile base
- **robot_arm** - MoveIt configuration for manipulation
- **robot_simulation** - Integrated simulation scenarios
- **robot_description** - Robot model definition

## Development

### Adding New Behaviors

1. Create a new Python script in `scripts/`
2. Implement behavior logic using action clients
3. Register behavior with mission node
4. Update package dependencies if needed

### Testing Behaviors

```bash
# Test navigation behavior
rosrun robot_behavior nav_action.py

# Test manipulation behavior
rosrun robot_behavior arm_action.py

# Run complete mission
rosrun robot_behavior main_mission.py
```
