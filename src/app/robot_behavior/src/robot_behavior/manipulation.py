import py_trees
import rospy
import moveit_commander
import sys

class MoveArmToState(py_trees.behaviour.Behaviour):
    """
    Behavior Tree Node to move the robot arm to a named target (e.g., 'home', 'ready')
    defined in the SRDF using MoveIt.
    """
    def __init__(self, name, target_state_name, group_name="arm"):
        super(MoveArmToState, self).__init__(name)
        self.target_state_name = target_state_name
        self.group_name = group_name
        self.move_group = None

    def setup(self, **kwargs):
        """
        Initialize the MoveGroupCommander.
        """
        try:
            # Note: roscpp_initialize should be called in the main script, 
            # but we can instantiate the commander here.
            self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
            return True
        except Exception as e:
            self.logger.error(f"[{self.name}] Failed to setup MoveIt: {e}")
            return False

    def initialise(self):
        """
        Plan and execute the motion.
        """
        self.logger.info(f"[{self.name}] Moving arm to '{self.target_state_name}'...")
        self.move_group.set_named_target(self.target_state_name)
        
        # Non-blocking call initially, we check status in update()
        # However, MoveIt's 'go()' is blocking by default. 
        # For a responsive tree, we should use async, but for simplicity in this demo
        # we will block briefly.
        self.success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def update(self):
        """
        Return success or failure based on the execution result.
        """
        if self.success:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass
