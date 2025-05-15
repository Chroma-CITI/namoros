import py_trees
from py_trees.common import Status
from rclpy import Future

from namoros.behavior_node import NamoBehaviorNode
from nav2_msgs.action._follow_path import FollowPath_GetResult_Response
from nav2_msgs.action._smooth_path import SmoothPath_GetResult_Response
from action_msgs.msg import GoalStatus
from namoros_msgs.msg import NamoPath
from nav_msgs.msg import Path


class FollowPath(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        node: NamoBehaviorNode,
        namo_path: NamoPath,
        is_evasion: bool = False,
    ):
        super().__init__(name="follow_path")
        self.node = node
        self.namo_path = namo_path
        self._status: Status = Status.INVALID
        self.is_evasion = is_evasion

    def smooth_path_callback(self, future: Future):
        try:
            result: SmoothPath_GetResult_Response = future.result()  # type: ignore
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                result.result.path
                follow_path_future = self.node.follow_path(
                    path=result.result.path,
                    controller_id=(
                        "TransferPath" if self.namo_path.is_transfer else "TransitPath"
                    ),
                )
                follow_path_future.add_done_callback(self.follow_path_callback)
            else:
                self._status = Status.FAILURE
                self.node.get_logger().error(f"SmoothPath failed with result: {result}")
        except Exception as e:
            self.node.get_logger().error(f"SmoothPath failed with error: {e}")
            self._status = Status.FAILURE

    def follow_path_callback(self, future: Future):
        try:
            result: FollowPath_GetResult_Response = future.result()  # type: ignore
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self._status = Status.SUCCESS
            else:
                self._status = Status.FAILURE
                self.node.get_logger().info(f"FollowPath failed with result: {result}")
                self.node.trigger_a_replan()
        except Exception as e:
            self.logger.error("FollowPath failed with error: {e}")
            self._status = Status.FAILURE

    def smooth_and_follow_path(self, path: Path):
        smooth_path_future = self.node.smooth_path(path=path)
        smooth_path_future.add_done_callback(self.smooth_path_callback)

    def initialise(self):
        self._status = Status.RUNNING
        self.smooth_and_follow_path(self.namo_path.path)

    def update(self):
        if self._status == Status.RUNNING:
            self.node.get_logger().info(
                "Following path." if not self.is_evasion else "Evading."
            )
        self.status = self._status
        self.node.publish_status_marker(
            "FOLLOWING PATH" if not self.is_evasion else "EVADING"
        )
        return self.status
