#!/usr/bin/env python3
import sys
import py_trees
import rospy
import moveit_commander
import geometry_msgs.msg

class MoveArmAction(py_trees.behaviour.Behaviour):
    """
    A Behavior Tree Leaf Node that moves the robot arm to a named target (e.g., 'home', 'ready').
    Uses MoveIt Commander.
    """
    def __init__(self, name, target_name):
        super(MoveArmAction, self).__init__(name)
        self.target_name = target_name
        self.group_name = "arm"
        self.move_group = None

    def setup(self, **kwargs):
        """
        Initialize MoveIt Commander.
        """
        # MoveIt commander needs to be initialized once in the main process, 
        # but we can grab the group here.
        # Note: In a real app, ensure moveit_commander.roscpp_initialize(sys.argv) is called in main.
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        return True

    def initialise(self):
        """
        Plan and execute the motion.
        """
        rospy.loginfo(f"[{self.name}] Moving arm to target: {self.target_name}")
        
        # Set the named target (defined in SRDF)
        self.move_group.set_named_target(self.target_name)
        
        # Plan and execute (non-blocking here would require more complex logic, 
        # but for simplicity we block until done or use the go() command which blocks by default)
        success = self.move_group.go(wait=True)
        
        # Stop any residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if success:
            self.feedback_message = "Movement successful"
            self.result = True
        else:
            self.feedback_message = "Movement failed"
            self.result = False

    def update(self):
        """
        Since move_group.go(wait=True) is blocking, by the time we get here, it's done.
        In a more advanced version, we would use async execution and check status here.
        """
        if self.result:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass
