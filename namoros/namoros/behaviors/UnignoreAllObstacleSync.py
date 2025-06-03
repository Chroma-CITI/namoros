from namoros.behavior_node import NamoBehaviorNode
import py_trees


class UnignoreAllObstacleSync(py_trees.behaviour.Behaviour):
    def __init__(self, node: NamoBehaviorNode):
        super().__init__(name="UnignoreAllObstacleSync")
        self.node = node

    def initialise(self):
        self.node.state.unignore_all()

    def update(self):
        return py_trees.common.Status.SUCCESS
