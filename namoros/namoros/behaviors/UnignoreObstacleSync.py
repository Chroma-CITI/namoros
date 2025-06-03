from namoros.behavior_node import NamoBehaviorNode
import py_trees


class IgnoreObstacleSync(py_trees.behaviour.Behaviour):
    def __init__(
        self, node: NamoBehaviorNode, obstacle_id: str, unignore: bool = False
    ):
        super().__init__(name="IgnoreObstacleSync")
        self.node = node
        self.obstacle_id = obstacle_id
        self.unignore = unignore

    def initialise(self):
        if self.unignore:
            self.node.world_state_tracker.unignore_obstacle(self.obstacle_id)
        else:
            self.node.world_state_tracker.ignore_obstacle(self.obstacle_id)

    def update(self):
        return py_trees.common.Status.SUCCESS
