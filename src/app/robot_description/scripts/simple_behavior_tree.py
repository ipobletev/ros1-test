#!/usr/bin/env python3
import rospy
import py_trees
import py_trees_ros
import geometry_msgs.msg
import std_msgs.msg
from py_trees.common import Status

"""
BEHAVIOR TREE TUTORIAL FOR ROS1
-------------------------------
This script demonstrates how to create a Behavior Tree (BT) to control a robot.
A BT is a tree of nodes where:
- The 'Root' ticks its children.
- 'Composites' (Sequence, Selector) decide which child to run.
- 'Leaves' (Actions, Conditions) do the actual work.

Scenario:
The robot will infinitely loop through:
1. Go to Waypoint A
2. Wait/Look around
3. Go to Waypoint B
"""

class MoveToLocation(py_trees.behaviour.Behaviour):
    """
    A Custom Action Node to move the robot to a specific (x, y) location.
    In a real robot, this would send a goal to the 'move_base' action server.
    Here, we simulate it by publishing to /cmd_vel for a short duration.
    """
    def __init__(self, name, target_x, target_y):
        super(MoveToLocation, self).__init__(name)
        self.target_x = target_x
        self.target_y = target_y
        self.publisher = None
        self.counter = 0

    def setup(self, **kwargs):
        """Called once when the tree is initialized."""
        self.publisher = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        return True

    def initialise(self):
        """Called every time this node starts running (becomes ACTIVE)."""
        self.counter = 0
        rospy.loginfo(f"[{self.name}] Moving to ({self.target_x}, {self.target_y})...")

    def update(self):
        """
        Called every 'tick' of the tree while this node is ACTIVE.
        Returns: RUNNING, SUCCESS, or FAILURE.
        """
        # Simulate movement logic
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = 0.5 # Move forward
        self.publisher.publish(msg)

        self.counter += 1
        # Simulate reaching the goal after 20 ticks
        if self.counter > 20:
            msg.linear.x = 0.0
            self.publisher.publish(msg)
            rospy.loginfo(f"[{self.name}] Arrived!")
            return Status.SUCCESS
        
        return Status.RUNNING

    def terminate(self, new_status):
        """Called when the node stops running."""
        pass

def create_tree():
    """
    Builds the Behavior Tree structure.
    Structure:
      - Sequence (Root)
        - MoveToLocation (A)
        - MoveToLocation (B)
    """
    # The Root Node: A Sequence runs children in order. 
    # If one fails, the sequence fails. If one succeeds, it runs the next.
    root = py_trees.composites.Sequence("Patrol Sequence", memory=True)

    # Create the action nodes
    task_a = MoveToLocation("Go to Point A", 1.0, 0.0)
    task_b = MoveToLocation("Go to Point B", 2.0, 2.0)

    # Add children to the root
    root.add_children([task_a, task_b])
    
    return root

if __name__ == '__main__':
    rospy.init_node("behavior_tree_node")
    
    # 1. Create the tree
    root = create_tree()

    # 2. Create a BehaviourTree engine to tick the tree
    tree = py_trees_ros.trees.BehaviourTree(root)
    
    # 3. Setup connections (publishers/subscribers)
    tree.setup(timeout=15)

    # 4. Tick the tree periodically (e.g., every 100ms)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tree.tick()
        rate.sleep()
