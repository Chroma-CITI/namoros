import py_trees
from namoros.behavior_node import NamoBehaviorNode
from py_trees.common import Status


class ManualSyncPlanner(py_trees.behaviour.Behaviour):
    def __init__(
        self, node: NamoBehaviorNode, path_index: int = -1, action_index: int = -1
    ):
        super().__init__(name="ManualSyncPlanner")
        self.node = node
        self.path_index = path_index
        self.action_index = action_index

    def update(self):
        self.node.synchronize_planner(
            path_index=self.path_index, action_index=self.action_index
        )
        return Status.SUCCESS
