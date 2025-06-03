import py_trees
from py_trees.common import Status
from rclpy.action import ActionClient
from namoros_msgs.action import UpdatePlan as UpdatePlanMsg
from namoros.behavior_node import NamoBehaviorNode
from rclpy import Future
from rclpy.action.client import ClientGoalHandle
from rclpy.action import GoalResponse

import typing as t


class UpdatePlan(py_trees.behaviour.Behaviour):
    def __init__(self, node: NamoBehaviorNode):
        super().__init__(name="UpdatePlan")
        self.node = node
        self._status: Status = Status.INVALID
        self._update_plan_client = ActionClient(
            node=self.node,
            action_type=UpdatePlanMsg,
            action_name="namo_planner/update_plan",
        )

    def _send_goal(self):
        goal = UpdatePlanMsg.Goal()
        self.future = self._update_plan_client.send_goal_async(goal)
        self.future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future: Future):
        # Handle the server's response to the goal request
        goal_handle = t.cast(ClientGoalHandle | None, future.result())
        if not goal_handle or not goal_handle.accepted:
            self.node.get_logger().info("UpdatePlan goal rejected by server")
            self._status = Status.FAILURE
            return

        # Request the result asynchronously
        result: Future = goal_handle.get_result_async()
        result.add_done_callback(self._result_callback)

    def _result_callback(self, future: Future) -> None:
        try:
            goal_response = t.cast(GoalResponse, future.result())
            result: UpdatePlanMsg.Result = goal_response.result
            if result.plan is None or len(result.plan.paths) == 0:
                self._status = Status.RUNNING
                self.node.get_logger().info("Failed to update the plan")
                self.node.trigger_a_replan()
            else:
                self.node.state.plan = result.plan
                self._status = Status.SUCCESS
        except Exception as e:
            self.node.get_logger().error(f"An error occured while updating plan: {e}")
            self._status = Status.FAILURE

    def initialise(self):
        if not self.node.state.goal_pose:
            raise Exception("No goal pose")
        self._status = Status.RUNNING
        self._send_goal()

    def update(self):
        self.node.get_logger().info("UPDATING PLAN")
        self.node.publish_status_marker("UPDATING PLAN")
        return self._status
