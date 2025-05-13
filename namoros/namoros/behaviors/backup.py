import time
import py_trees
from py_trees.common import Status
from rclpy import Future
from nav2_msgs.action._back_up import BackUp_GetResult_Response
from namoros.behavior_node import NamoBehaviorNode
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist


class BackUp(py_trees.behaviour.Behaviour):
    def __init__(self, node: NamoBehaviorNode, distance: float):
        super().__init__(name="BackUp")
        self.node = node
        self.distance = distance
        self.speed = 0.05
        self.duration = distance / self.speed
        self._status: Status = Status.INVALID
        self.start_time = 0

    def callback(self, future: Future):
        try:
            result: BackUp_GetResult_Response = future.result()  # type: ignore
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self._status = Status.SUCCESS
            else:
                self._status = Status.FAILURE
                self.node.get_logger().info(f"BackUp failed with result: {result}")
        except Exception as e:
            self.node.get_logger().error(f"BackUp failed with error: {e}")
            self._status = Status.FAILURE

    def initialise(self):
        self._status = Status.RUNNING
        # future = self.node.backup(self.distance)
        # future.add_done_callback(self.callback)
        self.start_time = time.time()
        self.publish_velocity_cmd(-self.speed)

    def publish_velocity_cmd(self, speed: float):
        cmd_vel = Twist()
        cmd_vel.linear.x = float(speed)
        cmd_vel.angular.z = 0.0
        self.node.publish_cmd_vel(cmd_vel)

    def update(self):
        elapsed = time.time() - self.start_time
        if elapsed <= self.duration:
            self.node.publish_status_marker(
                f"BACKING UP ({elapsed:.2f}/{self.duration:.2f})"
            )
            self.node.get_logger().info(
                f"BACKING UP ({elapsed:.2f}/{self.duration:.2f})"
            )
            self.publish_velocity_cmd(-self.speed)
            return Status.RUNNING
        self.publish_velocity_cmd(0)  # stop
        return Status.SUCCESS
