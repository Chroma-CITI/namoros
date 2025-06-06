from namoros.behavior_node import NamoBehaviorNode
import py_trees


class UnignoreObstacleSync(py_trees.behaviour.Behaviour):
    def __init__(self, node: NamoBehaviorNode, obstacle_id: str):
        super().__init__(name="UnignoreObstacleSync")
        self.node = node
        self.obstacle_id = obstacle_id

    def initialise(self):
        self.node.state.unignore_obstacle(self.obstacle_id)

    def update(self):
        return py_trees.common.Status.SUCCESS
