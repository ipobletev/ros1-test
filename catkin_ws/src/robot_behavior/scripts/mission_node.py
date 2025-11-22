#!/usr/bin/env python3
import sys
import rospy
import py_trees
import py_trees_ros
import moveit_commander

# IMPORTING OUR CUSTOM LIBRARY
from robot_behavior.navigation import MoveToPose
from robot_behavior.manipulation import MoveArmToState

def create_root():
    """
    Defines the mission behavior.
    """
    # Root Sequence: Run children in order
    root = py_trees.composites.Sequence("Main Mission", memory=True)

    # 1. Start with Arm at Home
    step_1 = MoveArmToState("Init Arm", "home")

    # 2. Go to Pickup Zone (x=2.0, y=0.0)
    step_2 = MoveToPose("Go to Pickup", 2.0, 0.0)

    # 3. Prepare to Pick (Move arm to 'ready')
    step_3 = MoveArmToState("Prepare Grip", "ready")

    # 4. Return to Base (x=0.0, y=0.0)
    step_4 = MoveToPose("Return Base", 0.0, 0.0)

    # 5. Drop/Reset Arm
    step_5 = MoveArmToState("Reset Arm", "home")

    root.add_children([step_1, step_2, step_3, step_4, step_5])
    return root

if __name__ == '__main__':
    rospy.init_node("mission_controller")
    
    # Initialize MoveIt (Global initialization)
    moveit_commander.roscpp_initialize(sys.argv)

    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root)
    
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError:
        rospy.logerr("Timeout waiting for actions (move_base/moveit).")
        sys.exit(1)

    # Tick the tree at 10Hz
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tree.tick()
        rate.sleep()
