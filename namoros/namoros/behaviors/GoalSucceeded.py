import py_trees
from geometry_msgs.msg import Twist
from py_trees.common import Status
from namoros.behavior_node import NamoBehaviorNode


class GoalSucceeded(py_trees.behaviour.Behaviour):
    def __init__(self, node: NamoBehaviorNode):
        super().__init__(name="GoalSucceeded")
        self.node = node

    def update(self):
        self.node.state.init_next_goal()
        self.node.publish_status_marker(f"GOAL SUCCEEDED")
        return Status.SUCCESS
