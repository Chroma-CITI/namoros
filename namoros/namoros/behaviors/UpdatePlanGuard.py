import py_trees

from namoros.behavior_node import NamoBehaviorNode


class UpdatePlanGuard(py_trees.behaviour.Behaviour):
    def __init__(self, node: NamoBehaviorNode):
        super().__init__(name="UpdatePlanGuard")
        self.node = node

    def update(self):
        if self.node.state.is_update_plan_triggered():
            self.node.state.clear_update_plan_trigger()
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
