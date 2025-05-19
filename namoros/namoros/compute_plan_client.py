import rclpy
from rclpy.action import ActionClient
from namoros_msgs.action import UpdatePlan
from geometry_msgs.msg import PoseStamped, Header
from std_msgs.msg import Header
from namoros.behavior_node import NamoBehaviorNode


class ComputePlanClient:
    def __init__(self, node: NamoBehaviorNode):
        self.node = node
        self._compute_plan_client = ActionClient(
            self, ComputePlanClient, "compute_plan"
        )
        self._update_plan_client = ActionClient(self, UpdatePlan, "update_plan")

    def update_plan(self):
        # Create the goal message
        goal_msg = UpdatePlan.Goal()

        # Populate start_pose (example values)
        goal_msg.start_pose = PoseStamped()
        goal_msg.start_pose.header = Header(
            frame_id="map", stamp=self.get_clock().now().to_msg()
        )
        goal_msg.start_pose.pose.position.x = 0.0
        goal_msg.start_pose.pose.position.y = 0.0
        goal_msg.start_pose.pose.position.z = 0.0
        goal_msg.start_pose.pose.orientation.w = 1.0

        # Populate goal_pose (example values)
        goal_msg.goal_pose = PoseStamped()
        goal_msg.goal_pose.header = Header(
            frame_id="map", stamp=self.get_clock().now().to_msg()
        )
        goal_msg.goal_pose.pose.position.x = 5.0
        goal_msg.goal_pose.pose.position.y = 5.0
        goal_msg.goal_pose.pose.position.z = 0.0
        goal_msg.goal_pose.pose.orientation.w = 1.0

        # Wait for the action server to be available
        self.get_logger().info("Waiting for action server...")
        self._update_plan_client.wait_for_server()

        # Send the goal asynchronously
        self.get_logger().info("Sending goal...")
        self.future = self._update_plan_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Handle the server's response to the goal request
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected by server")
            return
        self.get_logger().info("Goal accepted by server")

        # Request the result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        # Handle the result
        result = future.result().result
        self.get_logger().info("Received result")
        # Example: Log some details from the NamoPlan result
        # Adjust based on the actual structure of NamoPlan.msg
        self.get_logger().info(
            f"Plan received with {len(result.plan.poses)} poses"
            if hasattr(result.plan, "poses")
            else "Plan received"
        )
        # Shutdown the node after receiving the result
        rclpy.shutdown()
