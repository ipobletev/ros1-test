#!/usr/bin/env python3
import py_trees
import py_trees_ros
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class MoveBaseActionClient(py_trees.behaviour.Behaviour):
    """
    A Behavior Tree Leaf Node that sends a goal to the move_base action server.
    """
    def __init__(self, name, x, y, frame_id="map"):
        super(MoveBaseActionClient, self).__init__(name)
        self.target_x = x
        self.target_y = y
        self.frame_id = frame_id
        self.client = None

    def setup(self, **kwargs):
        """
        Setup the Action Client.
        """
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo(f"[{self.name}] Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo(f"[{self.name}] Connected to move_base!")
        return True

    def initialise(self):
        """
        Called when the node starts. Sends the goal.
        """
        rospy.loginfo(f"[{self.name}] Sending goal: x={self.target_x}, y={self.target_y}")
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = self.target_x
        goal.target_pose.pose.position.y = self.target_y
        goal.target_pose.pose.orientation.w = 1.0 # No rotation logic for simplicity

        self.client.send_goal(goal)

    def update(self):
        """
        Checks the status of the action.
        """
        # Check if the action is done
        state = self.client.get_state()
        
        if state == actionlib.GoalStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        elif state == actionlib.GoalStatus.ABORTED or state == actionlib.GoalStatus.REJECTED:
            return py_trees.common.Status.FAILURE
        
        # If still running
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Called when the node stops. Cancel goal if we are not successful.
        """
        if new_status == py_trees.common.Status.INVALID:
            self.client.cancel_goal()
