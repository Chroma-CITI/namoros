import py_trees
from namoros.behavior_node import NamoBehaviorNode
from py_trees.common import Status


class SynchronizePlannerPeriodic(py_trees.behaviour.Behaviour):
    def __init__(self, node: NamoBehaviorNode):
        super().__init__(name="SynchronizePlannerPeriodic")
        self.node = node

    def initialise(self):
        self.status = Status.RUNNING

    def update(self):
        self.node.synchronize_planner()
        return self.status
