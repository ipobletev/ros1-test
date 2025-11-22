import py_trees
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class MoveToPose(py_trees.behaviour.Behaviour):
    """
    Behavior Tree Node to move the robot to a specific (x, y) location using move_base.
    """
    def __init__(self, name, target_x, target_y, frame_id="map"):
        super(MoveToPose, self).__init__(name)
        self.target_x = target_x
        self.target_y = target_y
        self.frame_id = frame_id
        self.client = None

    def setup(self, **kwargs):
        """
        Connect to the move_base action server.
        """
        self.logger.debug(f"[{self.name}] Connecting to move_base...")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not self.client.wait_for_server(timeout=rospy.Duration(5.0)):
            self.logger.error(f"[{self.name}] move_base server not found!")
            return False
        self.logger.debug(f"[{self.name}] Connected to move_base.")
        return True

    def initialise(self):
        """
        Send the goal when the node becomes active.
        """
        self.logger.info(f"[{self.name}] Navigating to ({self.target_x}, {self.target_y})...")
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.target_x
        goal.target_pose.pose.position.y = self.target_y
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)

    def update(self):
        """
        Check status while running.
        """
        state = self.client.get_state()

        if state == actionlib.GoalStatus.SUCCEEDED:
            self.logger.info(f"[{self.name}] Target reached.")
            return py_trees.common.Status.SUCCESS
        
        elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.LOST]:
            self.logger.warn(f"[{self.name}] Navigation failed.")
            return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Cancel navigation if the tree interrupts this node.
        """
        if new_status == py_trees.common.Status.INVALID:
            self.client.cancel_goal()
