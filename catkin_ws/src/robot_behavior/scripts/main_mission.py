#!/usr/bin/env python3
import sys
import rospy
import py_trees
import py_trees_ros
import moveit_commander

# Import our custom actions
from nav_action import MoveBaseActionClient
from arm_action import MoveArmAction

def create_mission_tree():
    """
    Creates a Sequence of tasks:
    1. Move Arm to 'home'
    2. Go to Point A
    3. Move Arm to 'ready' (simulate picking something)
    4. Go to Point B
    5. Move Arm to 'home' (simulate dropping)
    """
    # Root Sequence
    root = py_trees.composites.Sequence("Mission Root", memory=True)

    # 1. Initialize Arm
    arm_home_1 = MoveArmAction("Arm Home", "home")

    # 2. Navigate to Point A (x=2.0, y=0.0)
    nav_to_a = MoveBaseActionClient("Go to A", 2.0, 0.0)

    # 3. Action at A (Move arm to 'ready')
    arm_ready = MoveArmAction("Pick Object", "ready")

    # 4. Navigate to Point B (x=0.0, y=0.0) - Return to start
    nav_to_b = MoveBaseActionClient("Return to Base", 0.0, 0.0)

    # 5. Drop Object
    arm_home_2 = MoveArmAction("Drop Object", "home")

    # Add all to the sequence
    root.add_children([
        arm_home_1,
        nav_to_a,
        arm_ready,
        nav_to_b,
        arm_home_2
    ])

    return root

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node("mission_behavior_tree")
    
    # Initialize MoveIt (required for arm actions)
    moveit_commander.roscpp_initialize(sys.argv)

    # Create the tree
    root = create_mission_tree()

    # Create the ROS tree manager
    tree = py_trees_ros.trees.BehaviourTree(root)
    
    # Setup (connects to topics/actions)
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError:
        rospy.logerr("Failed to setup the tree (timeout connecting to actions).")
        sys.exit(1)

    # Tick the tree
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        tree.tick()
        rate.sleep()
