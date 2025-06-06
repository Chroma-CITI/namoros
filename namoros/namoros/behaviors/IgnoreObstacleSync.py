from namoros.behavior_node import NamoBehaviorNode
import py_trees


class IgnoreObstacleSync(py_trees.behaviour.Behaviour):
    def __init__(self, node: NamoBehaviorNode, obstacle_id: str):
        super().__init__(name="IgnoreObstacleSync")
        self.node = node
        self.obstacle_id = obstacle_id

    def update(self):
        self.node.state.ignore_obstacle(self.obstacle_id)
        return py_trees.common.Status.SUCCESS
