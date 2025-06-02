import py_trees
from namoros.behavior_node import NamoBehaviorNode
from py_trees.common import Status


class ManualSyncPlanner(py_trees.behaviour.Behaviour):
    def __init__(self, node: NamoBehaviorNode, current_action_index: int = -1):
        super().__init__(name="ManualSyncPlanner")
        self.node = node
        self.current_action_index = current_action_index

    def update(self):
        self.node.synchronize_planner(current_action_index=self.current_action_index)
        return Status.SUCCESS
