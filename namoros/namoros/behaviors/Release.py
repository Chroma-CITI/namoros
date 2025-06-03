import py_trees
from action_msgs.msg import GoalStatus
from nav2_msgs.action._compute_path_to_pose import ComputePathToPose_GetResult_Response
from nav2_msgs.action._follow_path import FollowPath_GetResult_Response
from nav2_msgs.action._smooth_path import SmoothPath_GetResult_Response
from nav_msgs.msg import Path
from py_trees.common import Status
from rclpy import Future

from namoros.behavior_node import NamoBehaviorNode


class Release(py_trees.behaviour.Behaviour):
    def __init__(self, node: NamoBehaviorNode):
        super().__init__(name="Release")
        self.node = node
        self._status: Status = Status.INVALID

    def follow_path_callback(self, future: Future):
        try:
            result: FollowPath_GetResult_Response = future.result()  # type: ignore
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self._status = Status.SUCCESS
            else:
                self._status = Status.FAILURE
                self.node.get_logger().error(f"FollowPath failed with result: {result}")
        except Exception as e:
            self.node.get_logger().error(f"FollowPath failed with error: {e}")
            self._status = Status.FAILURE

    def get_path_callback(self, future: Future):
        try:
            result: ComputePathToPose_GetResult_Response = future.result()  # type: ignore
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                result.result.path
                follow_path_future = self.node.follow_path(
                    path=result.result.path, controller_id="TransferPath"
                )
                follow_path_future.add_done_callback(self.follow_path_callback)
            else:
                self._status = Status.FAILURE
                self.node.get_logger().error(
                    f"ComputePath failed with result: {result}"
                )
        except Exception as e:
            self.node.get_logger().error(f"ComputePath failed with error: {e}")
            self._status = Status.FAILURE

    def smooth_path_callback(self, future: Future):
        try:
            result: SmoothPath_GetResult_Response = future.result()  # type: ignore
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                result.result.path
                follow_path_future = self.node.follow_path(
                    path=result.result.path, controller_id="TransferPath"
                )
                follow_path_future.add_done_callback(self.follow_path_callback)

            else:
                self._status = Status.FAILURE
                self.node.get_logger().error(f"SmoothPath failed with result: {result}")
        except Exception as e:
            self.node.get_logger().error(f"SmoothPath failed with error: {e}")
            self._status = Status.FAILURE

    def follow_path(self, path: Path):
        follow_path_future = self.node.follow_path(
            path=path, controller_id="TransferPath"
        )
        follow_path_future.add_done_callback(self.follow_path_callback)

    def initialise(self):
        self._status = Status.SUCCESS
        self.node.release()

    def update(self):
        self.status = self._status
        return self.status
