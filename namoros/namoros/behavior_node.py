import copy
import functools
import math
import random
import typing as t

from geometry_msgs.msg import Polygon, Point, Pose, PoseArray, PoseWithCovarianceStamped
from kobuki_ros_interfaces.msg import BumperEvent
from kobuki_ros_interfaces.msg._sound import Sound
import numpy as np
from namoros_msgs.msg._namo_plan import NamoPlan
from namoros.data_models import load_namoros_config
from namoros.movable_obstacle_tracker import MovableObstacleTracker
from namoros.navigator import BasicNavigator, GoalStatus
from namoros.utils import Pose2D
from nav2_msgs.action import (
    FollowPath,
    BackUp,
    DriveOnHeading,
    Wait,
    ComputePathToPose,
    SmoothPath,
    Spin,
)
from nav_msgs.msg import Path
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter as RosParam
from ros_gz_interfaces.msg import ParamVec
from builtin_interfaces.msg import Duration
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,  # type: ignore
    ConnectivityException,  # type: ignore
    ExtrapolationException,  # type: ignore
)
import rclpy.time
from tf2_geometry_msgs import PoseStamped
import namoros.utils as utils
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap
from numpy.linalg import LinAlgError
from rcl_interfaces.msg import Parameter, ParameterValue
import subprocess
import shapely.geometry as geom
from shapely import affinity
from namoros_msgs.srv import (
    AddOrUpdateMovableObstacle,
    SimulatePath,
    GetEntityPolygon,
    SynchronizeState,
    DetectConflicts,
)
from namoros_msgs.msg import NamoPath, NamoEntity, NamoConflict
from namosim.world.world import World
from namoros.world_state_tracker import WorldStateTracker
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from namosim.agents.stilman_2005_agent import Stilman2005Agent
from namoros_msgs.action import ComputePlan
from rclpy.clock import Clock


class NamoState:
    def __init__(self, agent: Stilman2005Agent, node: "NamoBehaviorNode", clock: Clock):
        self.agent = agent
        self.node = node
        self.clock = clock
        self.replan_count: int = 0
        self.publish_init_pose_count = 0
        self.goal_pose: PoseStamped | None = None
        self.goals = copy.deepcopy(agent._navigation_goals)
        self.reset()

    def reset(self):
        self._replan_flag: bool = False
        self._update_plan_flag: bool = False
        self.bumper_pressed: bool = False
        self.movable_obstacle_tracker = MovableObstacleTracker(self.node)
        self.forward_dist_to_obstacle: float = float("inf")
        self.plan: NamoPlan | None = None
        self.world_state_tracker = WorldStateTracker()
        self.ignored_obstacles: t.Set[str] = set()
        self.init_goal()

    def init_goal(self):
        self.goal_pose = None
        if self.goal_pose is None and len(self.goals) > 0:
            goal = self.goals[0]
            header = Header()
            header.frame_id = "map"
            header.stamp = self.clock.now().to_msg()
            self.goal_pose = utils.construct_ros_pose(
                x=goal.pose[0], y=goal.pose[1], z=0.0, theta=goal.pose[2], header=header
            )

    def init_next_goal(self):
        if len(self.goals):
            self.goals.pop(0)
        self.init_goal()

    def ignore_obstacle(self, obstacle_id: str):
        self.ignored_obstacles.add(obstacle_id)

    def unignore_obstacle(self, obstacle_id: str):
        if obstacle_id in self.ignored_obstacles:
            self.ignored_obstacles.remove(obstacle_id)

    def unignore_all(self):
        self.ignored_obstacles.clear()

    def get_obstacle_pose(self, obstacle_id: str) -> Pose | None:
        if obstacle_id not in self.world_state_tracker.obstacles:
            return None
        pose = self.world_state_tracker.obstacles[obstacle_id]
        return pose

    def trigger_a_replan(self):
        self.replan_count += 1
        self._replan_flag = True
        self._update_plan_flag = False
        self.world_state_tracker.clear()

    def is_replan_triggered(self) -> bool:
        return self._replan_flag

    def clear_replan_trigger(self):
        self._replan_flag = False

    def trigger_update_plan(self):
        # update plan is like replan but does reset the world state tracker
        self.replan_count += 1
        self._replan_flag = False
        self._update_plan_flag = True

    def is_update_plan_triggered(self) -> bool:
        return self._update_plan_flag

    def clear_update_plan_trigger(self):
        self._update_plan_flag = False


class NamoBehaviorNode(Node):
    def __init__(self):
        super().__init__("namo_behavior", parameter_overrides=[])
        self.declare_parameters(
            namespace="",
            parameters=[
                ("scenario_file", RosParam.Type.STRING),
                ("config_file", RosParam.Type.STRING),
                ("agent_id", RosParam.Type.STRING),
                ("omniscient_obstacle_perception", RosParam.Type.BOOL),
            ],
        )
        self.declare_parameter("is_sim", False)
        self.scenario_file = t.cast(str, self.get_parameter("scenario_file").value)
        self.agent_id = t.cast(str, self.get_parameter("agent_id").value)
        self.namoros_config = load_namoros_config(
            t.cast(str, self.get_parameter("config_file").value)
        )
        self.omniscient_obstacle_perception = t.cast(
            bool, self.get_parameter("omniscient_obstacle_perception").value
        )
        self.is_sim = t.cast(bool, self.get_parameter("is_sim").value)

        if self.scenario_file.strip().lower().endswith(".svg"):
            world = World.load_from_svg(self.scenario_file)
        else:
            world = World.load_from_yaml(self.scenario_file)
        self.agent: Stilman2005Agent = t.cast(
            Stilman2005Agent, world.agents[self.agent_id]
        )
        self.robot_radius = world.agents[self.agent_id].circumscribed_radius

        self.main_cb_group = MutuallyExclusiveCallbackGroup()
        self.namo_cb_group = ReentrantCallbackGroup()

        # actions
        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            "follow_path",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.backup_client = ActionClient(self, BackUp, "backup")
        self.drive_on_heading_client = ActionClient(
            self, DriveOnHeading, "drive_on_heading"
        )
        self.spin_client = ActionClient(self, Spin, "spin")
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose"
        )
        self.smooth_path_client = ActionClient(self, SmoothPath, "smooth_path")
        self.wait_client = ActionClient(self, Wait, "wait")

        # subscribers
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, f"goal_pose_namo", self.goal_pose_callback, 1
        )
        self.sub_bumper = self.create_subscription(
            BumperEvent, "events/bumper", self.bumper_callback, 1
        )
        self.sub_laser_scan = self.create_subscription(
            LaserScan, "scan", self.laser_scan_callback, 1
        )
        self.sub_robot_info = self.create_subscription(
            NamoEntity, "/namo/robots", self._robot_info_callback, 1
        )
        self.sub_grab = self.create_subscription(
            ParamVec, "/namo_grab", self._namo_grab_callback, 1
        )
        self.sub_release = self.create_subscription(
            ParamVec, "/namo_release", self._namo_release_callback, 1
        )

        if self.is_sim:
            # subscriptions
            self.obstacle_pose_subscriptions = {}
            for obstacle in self.namoros_config.obstacles:
                self.obstacle_pose_subscriptions[obstacle.name] = (
                    self.create_subscription(
                        PoseArray,
                        f"/model/{obstacle.name}/pose",
                        self.create_obstacle_pose_callback(obstacle.name),
                        1,
                        callback_group=self.main_cb_group,
                    )
                )

        # publishers
        self.pub_init_pose = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 1
        )
        self.grab_publisher = self.create_publisher(ParamVec, "/namo_grab", 1)
        self.release_publisher = self.create_publisher(ParamVec, "/namo_release", 1)
        self.pub_sound = self.create_publisher(Sound, "/commands/sound", 1)
        self.local_footprint_publisher = self.create_publisher(
            Polygon, "local_costmap/footprint", 1
        )
        self.global_footprint_publisher = self.create_publisher(
            Polygon, "global_costmap/footprint", 1
        )
        self.path_publisher = self.create_publisher(Path, "current_namo_path", 1)
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 1)
        self.pub_robot_info = self.create_publisher(NamoEntity, "/namo/robots", 1)
        self.robot_info_timer = self.create_timer(
            1 / 2.0, self.publish_robot_info, MutuallyExclusiveCallbackGroup()
        )
        self.pub_status = self.create_publisher(
            Marker, "robot_status", 1, callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.nav = BasicNavigator(namespace=self.get_namespace())
        self.add_or_update_movable_obstacle_cb_group = MutuallyExclusiveCallbackGroup()

        # services
        self.srv_clear_local_costmap = self.create_client(
            ClearEntireCostmap, "local_costmap/clear_entirely_local_costmap"
        )
        self.srv_clear_global_costmap = self.create_client(
            ClearEntireCostmap, "global_costmap/clear_entirely_global_costmap"
        )
        self.srv_add_movable_obstacle = self.create_client(
            AddOrUpdateMovableObstacle,
            "namo_planner/add_or_update_movable_obstacle",
            callback_group=self.add_or_update_movable_obstacle_cb_group,
        )
        self.sync_cb_group = ReentrantCallbackGroup()
        self.srv_synchronize_planner = self.create_client(
            SynchronizeState,
            "namo_planner/synchronize_state",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.srv_detect_conflicts = self.create_client(
            DetectConflicts,
            "namo_planner/detect_conflicts",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # transform listener
        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=20))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # publish initial pose
        self.get_logger().info(f"Namespace: {self.get_namespace()}")
        self.publish_initial_pose_timer = self.create_timer(
            5.0, self.publish_initial_pose
        )
        self.publish_init_pose_count = 0

        # initialize state
        self.state = NamoState(agent=self.agent, node=self, clock=self.get_clock())
        self.goal_handle: ClientGoalHandle | None = None

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()

        # Set the header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # Adjust frame_id as needed

        # Set the pose (example values: x=0.0, y=0.0, z=0.0, yaw=0.0)
        msg.pose.pose.position.x = self.agent.pose[0]
        msg.pose.pose.position.y = self.agent.pose[1]
        msg.pose.pose.position.z = 0.0

        orientation = utils.euler_to_quat(0, 0, math.radians(self.agent.pose[2]))
        msg.pose.pose.orientation.x = orientation[0]
        msg.pose.pose.orientation.y = orientation[1]
        msg.pose.pose.orientation.z = orientation[2]
        msg.pose.pose.orientation.w = orientation[3]

        variance_xy = 0.01  # Standard deviation for x, y (meters)
        variance_yaw = 0.01  # Standard deviation for yaw (radians)

        # Set covariance (example: low uncertainty)
        msg.pose.covariance = np.zeros(36)  # 6x6 matrix flattened
        msg.pose.covariance[0] = variance_xy**2  # Variance in x
        msg.pose.covariance[7] = variance_xy**2  # Variance in y
        msg.pose.covariance[35] = variance_yaw**2  # Variance in yaw

        self.pub_init_pose.publish(msg)
        self.get_logger().info(
            "Published initial pose: x={}, y={}".format(
                msg.pose.pose.position.x, msg.pose.pose.position.y
            )
        )

        self.publish_init_pose_count += 1
        if self.publish_init_pose_count > 1:
            self.publish_initial_pose_timer.cancel()

    def reset(self):
        self.state.reset()

    def create_obstacle_pose_callback(self, obstacle_name: str):
        def cb(msg: PoseArray):
            pose = msg.poses[0]  # type: ignore
            self.state.world_state_tracker.update_obstacle(obstacle_name, pose)

        return cb

    def trigger_a_replan(self):
        self.get_logger().info("Triggering a replan.")
        self.state.trigger_a_replan()

    def trigger_update_plan(self):
        self.get_logger().info("Triggering a plan update.")
        self.state.trigger_update_plan()

    def laser_scan_callback(self, data: LaserScan):
        total_len = len(data.ranges)
        center_index = int(round((total_len / 2.0) - 1))
        min_dist = float("inf")
        values_around = 5
        for i in range(0, values_around + 1):
            if min_dist > data.ranges[center_index + i]:
                min_dist = data.ranges[center_index + i]
            if min_dist > data.ranges[center_index - i]:
                min_dist = data.ranges[center_index - i]
        self.forward_dist_to_obstacle = min_dist

    def _robot_info_callback(self, entity: NamoEntity):
        if entity.entity_id == self.agent_id:
            return
        self.state.world_state_tracker.update_robot(entity)

    def _namo_grab_callback(self, msg: ParamVec):
        robot_name: str | None = None
        obstacle_name: str | None = None
        for param in msg.params:
            param = t.cast(Parameter, param)
            if param.name == "robot_name":
                robot_name = param.value.string_value
            elif param.name == "obstacle_name":
                obstacle_name = param.value.string_value

        if robot_name and robot_name != self.agent_id and obstacle_name is not None:
            self.state.world_state_tracker.grab_obstacle(
                robot_id=robot_name, obstacle_id=obstacle_name
            )

    def _namo_release_callback(self, msg: ParamVec):
        robot_name: str | None = None
        obstacle_name: str | None = None
        for param in msg.params:
            param = t.cast(Parameter, param)
            if param.name == "robot_name":
                robot_name = param.value.string_value
            elif param.name == "obstacle_name":
                obstacle_name = param.value.string_value

        if robot_name and robot_name != self.agent_id and obstacle_name is not None:
            self.state.world_state_tracker.release_obstacle(robot_id=robot_name)

    def publish_robot_info(self):
        entity = NamoEntity()
        entity.entity_id = self.agent_id

        robot_pose = self.lookup_robot_pose()
        if not robot_pose:
            return

        robot_pose = utils.entity_pose_to_pose2d(robot_pose.pose)
        robot_polygon = self.lookup_robot_polygon()
        robot_polygon = utils.shapely_to_ros_polygon(robot_polygon)

        entity.pose.x = robot_pose.x
        entity.pose.y = robot_pose.y
        entity.pose.angle_degrees = robot_pose.degrees
        entity.polygon = robot_polygon

        self.pub_robot_info.publish(entity)

    def publish_cmd_vel(self, cmd_vel: Twist):
        self.pub_cmd_vel.publish(cmd_vel)

    def publish_namo_path(self, path: Path):
        self.clear_namo_path()
        self.path_publisher.publish(path)

    def clear_namo_path(self):
        empty_path = Path()
        empty_path.header.frame_id = "map"
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(empty_path)

    def transform_pose(
        self, input_pose: PoseStamped, target_frame: str
    ) -> PoseStamped | None:
        try:
            # Transform the pose to the target frame
            transformed_pose = self.tf_buffer.transform(
                input_pose, target_frame, timeout=rclpy.time.Duration(seconds=1)
            )
            return t.cast(PoseStamped, transformed_pose)
        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException,
            LinAlgError,
        ) as e:
            self.get_logger().error(f"Failed to transform pose: {e}")
            return None

    def _goal_response_callback(self, future: Future, final_result_future: Future):
        goal_handle: ClientGoalHandle = future.result()  # type: ignore
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected {goal_handle}")
            final_result_future.set_exception(Exception("goal rejected"))
            return

        self.goal_handle = goal_handle
        self.get_result_future: Future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(
            functools.partial(
                self._get_result_callback, final_result_future=final_result_future
            )
        )

    def _get_result_callback(self, future: Future, final_result_future: Future):
        try:
            result = future.result()
            final_result_future.set_result(result)
        except Exception as ex:
            self.get_logger().warn(f"action failed {ex}")
            final_result_future.set_exception(ex)

    def _wrap_send_goal_future(self, send_goal_future: Future):
        final_result_future = Future()
        send_goal_future.add_done_callback(
            functools.partial(
                self._goal_response_callback,
                final_result_future=final_result_future,
            )
        )
        return final_result_future

    def compose_compute_plan_goal_msg(self) -> ComputePlan.Goal:
        if not self.state.goal_pose:
            raise Exception("No goal pose")
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                source_frame=f"base_link",
                target_frame="map",
                time=rclpy.time.Time(seconds=0),
            )
        except Exception as ex:
            self.get_logger().warn(f"Failed to lookup robot tf: {ex}")
            raise ex
        robot_pose = utils.transform_to_pose(robot_tf)

        goal = ComputePlan.Goal()
        goal.start_pose = robot_pose
        goal.goal_pose = self.state.goal_pose
        self.get_logger().info(
            f"{self.agent_id} computing plan to goal {self.state.goal_pose}"
        )
        return goal

    def add_or_update_movable_obstable(
        self, uid: str, pose: Pose2D, polygon: geom.Polygon
    ):
        self.get_logger().info("Adding obstacle")
        req = AddOrUpdateMovableObstacle.Request()
        req.polygon = utils.shapely_to_ros_polygon(polygon)
        req.pose.x = float(pose.x)
        req.pose.y = float(pose.y)
        req.pose.angle_degrees = float(pose.degrees)
        req.obstacle_id = uid
        res: AddOrUpdateMovableObstacle.Response = self.srv_add_movable_obstacle.call(
            req
        )
        self.get_logger().info("Done adding obstacle")

    def synchronize_planner(self, path_index: int = -1, action_index: int = -1):
        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            self.get_logger().warn("Failed to lookup robot pose")
            return
        robot_pose = utils.entity_pose_to_pose2d(robot_pose.pose)
        req = SynchronizeState.Request()
        req.path_index = path_index
        req.action_index = action_index
        req.observed_robot_pose.x = robot_pose.x
        req.observed_robot_pose.y = robot_pose.y
        req.observed_robot_pose.angle_degrees = robot_pose.degrees

        other_robots: t.List[NamoEntity] = []
        for other_robot in self.state.world_state_tracker.robots.values():
            if (
                other_robot.entity_id
                in self.state.world_state_tracker.robot_to_obstacle
            ):
                other_robot.holding_other_entity_id = (
                    self.state.world_state_tracker.robot_to_obstacle[
                        other_robot.entity_id
                    ]
                )
            other_robots.append(other_robot)
        req.other_observed_robots = other_robots

        if self.omniscient_obstacle_perception:
            observed_obstacles: t.List[NamoEntity] = []
            for obs_id, obs_pose in self.state.world_state_tracker.obstacles.items():
                if obs_id in self.state.ignored_obstacles:
                    continue
                pose2d = utils.entity_pose_to_pose2d(obs_pose)
                obs_msg = NamoEntity()
                obs_msg.entity_id = obs_id
                obs_msg.pose.x = pose2d.x
                obs_msg.pose.y = pose2d.y
                obs_msg.pose.angle_degrees = pose2d.degrees
                observed_obstacles.append(obs_msg)
            req.observed_obstacles = observed_obstacles

        self.srv_synchronize_planner.call(req)

    def detect_conflicts(self):
        req = DetectConflicts.Request()
        res: DetectConflicts.Response = self.srv_detect_conflicts.call(req)
        return res.conflicts

    def lookup_robot_pose(self) -> PoseStamped | None:
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                source_frame=f"base_link",
                target_frame="map",
                time=rclpy.time.Time(seconds=0),
            )
        except Exception as ex:
            self.get_logger().warn(f"Failed to lookup robot tf: {ex}")
            return None
        robot_pose = utils.transform_to_pose(robot_tf)
        return robot_pose

    def lookup_robot_polygon(self) -> geom.Polygon:
        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            raise Exception("Failed to get robot pose")
        return t.cast(
            geom.Polygon,
            geom.Point(robot_pose.pose.position.x, robot_pose.pose.position.y).buffer(
                self.robot_radius
            ),
        )

    def lookup_pose(self, frame_id: str) -> PoseStamped | None:
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                source_frame=frame_id,
                target_frame="map",
                time=rclpy.time.Time(seconds=0),
            )
        except Exception as ex:
            self.get_logger().warn(f"Failed to lookup pose for frame {frame_id}: {ex}")
            return None
        pose = utils.transform_to_pose(robot_tf)
        return pose

    def goal_pose_callback(self, msg: PoseStamped):
        self.state.goal_pose = msg  # entity_pose_to_pose2d(msg.pose)
        self.get_logger().info(
            f"Received goal pose: {self.state.goal_pose}, frame: {msg.header.frame_id}"
        )

    def wait_for_successful_task_completion(self):
        while not self.nav.isTaskComplete():
            pass

        if not self.nav.status == GoalStatus.STATUS_SUCCEEDED:
            raise Exception("Failed to run nav task")

    def obstacle_marker_id_to_name(self, marker_id: str):
        for obs in self.namoros_config.obstacles:
            if obs.marker_id == marker_id:
                return obs.name
        return None

    def grab(self, obs_marker_id: str):
        self.get_logger().info(f"Grabbing obstacle {obs_marker_id}.")
        obstacle_name = self.obstacle_marker_id_to_name(obs_marker_id)
        if obstacle_name is None:
            obstacle_name = obs_marker_id
        if self.is_sim:
            self.set_obstacle_pose(obstacle_name=obstacle_name)

        params = ParamVec()
        robot_id_param = Parameter(name="robot_name")
        robot_id_param.value = ParameterValue(string_value=self.agent_id, type=4)
        robot_link_param = Parameter(name="robot_link")
        robot_link_param.value = ParameterValue(string_value="base_link", type=4)
        obs_id_param = Parameter(name="obstacle_name")
        obs_id_param.value = ParameterValue(string_value=obstacle_name, type=4)
        obs_link_param = Parameter(name="obstacle_link")
        obs_link_param.value = ParameterValue(string_value="box", type=4)
        params.params = [
            robot_id_param,
            robot_link_param,
            obs_id_param,
            obs_link_param,
        ]
        self.get_logger().info(f"Publishing params {str(params.__slots__)}.")
        self.grab_publisher.publish(params)
        self.update_robot_footprint_for_grab(obs_marker_id=obs_marker_id)
        self.state.world_state_tracker.grab_obstacle(
            robot_id=self.agent_id, obstacle_id=obstacle_name
        )
        self.get_logger().info(f"Grabbed obstacle {obs_marker_id}.")

    def release(self):
        self.get_logger().info(f"Release.")
        if self.is_sim:
            params = ParamVec()
            robot_id = Parameter(name="robot_name")
            robot_id.value = ParameterValue(string_value=self.agent_id, type=4)
            robot_link = Parameter(name="robot_link")
            robot_link.value = ParameterValue(string_value="base_link", type=4)
            params.params = [robot_id, robot_link]
            self.release_publisher.publish(params)
        self.update_robot_footprint_for_release()
        self.state.world_state_tracker.release_obstacle(robot_id=self.agent_id)

    def pose_info_callback(self, msg: PoseArray):
        for pose in msg.poses:
            self.get_logger().info(str(pose))

    def set_obstacle_pose(self, obstacle_name: str):
        box_pose = self.state.get_obstacle_pose(obstacle_name)
        if box_pose is None:
            return
        new_box_pose = copy.deepcopy(box_pose)
        new_box_pose.position.z += 0.4

        # Define the shell command
        pose_str = f"{{x: {new_box_pose.position.x}, y: {new_box_pose.position.y}, z: {new_box_pose.position.z}}}"
        orient_str = f"{{w: {new_box_pose.orientation.w}, x: {new_box_pose.orientation.x}, y: {new_box_pose.orientation.y}, z: {new_box_pose.orientation.z}}}"

        shell_command = f"ign service -s /world/namo_world/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 300 --req 'name: \"{obstacle_name}\", position: {pose_str}, orientation: {orient_str}'"
        self.get_logger().info(f"set obstacle pose command: {shell_command}")
        # Execute the shell command synchronously
        subprocess.run(shell_command, shell=True)

    def follow_path(self, path: Path, controller_id: str):
        """Returns a future that should resolve to a FollowPath.Result object"""
        self.publish_namo_path(path)
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = controller_id
        goal_msg.goal_checker_id = ""
        send_goal_future = self.follow_path_client.send_goal_async(goal_msg)
        final_result_future = self._wrap_send_goal_future(send_goal_future)
        return final_result_future

    def cancel_nav_task(self) -> Future:
        if self.goal_handle:
            self.get_logger().info("Canceling current task.")
            future = self.goal_handle.cancel_goal_async()
        else:
            future = Future()
            future.set_result(None)
        return future

    def advance(self, distance: float, speed: float = 0.05):
        """Returns a future that should resolve to a DriveOnHeading.Result object"""
        goal_msg = DriveOnHeading.Goal()
        goal_msg.target = Point(x=float(distance))
        goal_msg.speed = speed
        goal_msg.time_allowance = Duration(sec=20)
        send_goal_future = self.drive_on_heading_client.send_goal_async(goal_msg)
        final_result_future = self._wrap_send_goal_future(send_goal_future)
        return final_result_future

    def backup(self, distance: float, speed: float = 0.05):
        """Returns a future that should resolve to a BackUp.Result object"""
        goal_msg = BackUp.Goal()
        goal_msg.target = Point(x=float(distance))
        goal_msg.speed = speed
        goal_msg.time_allowance = Duration(sec=20)
        send_goal_future = self.backup_client.send_goal_async(goal_msg)
        final_result_future = self._wrap_send_goal_future(send_goal_future)
        return final_result_future

    def spin(self, target_yaw: float):
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = target_yaw
        goal_msg.time_allowance = Duration(sec=10)
        send_goal_future = self.spin_client.send_goal_async(goal_msg)
        final_result_future = self._wrap_send_goal_future(send_goal_future)
        return final_result_future

    def clear_bumper(self):
        self.bumper_pressed = False

    def request_bumper(self):
        self.play_random_sound()

    def play_sound(self, sound: int):
        self.pub_sound.publish(Sound(value=sound))

    def play_random_sound(self):
        random_sound = random.choice(
            [
                Sound.BUTTON,
                Sound.CLEANINGEND,
                Sound.CLEANINGSTART,
                Sound.ERROR,
                Sound.OFF,
                Sound.ON,
                Sound.RECHARGE,
            ]
        )
        self.pub_sound.publish(Sound(value=random_sound))

    def bumper_callback(self, event: BumperEvent):
        if event.state == BumperEvent.PRESSED:
            self.bumper_pressed = True
        else:
            self.bumper_pressed = False

    def compute_path(self, start: PoseStamped, goal: PoseStamped):
        msg = ComputePathToPose.Goal()
        msg.start = start
        msg.goal = goal
        msg.use_start = True
        send_goal_future = self.compute_path_client.send_goal_async(msg)
        final_result_future = self._wrap_send_goal_future(send_goal_future)
        return final_result_future

    def smooth_path(self, path: Path):
        msg = SmoothPath.Goal()
        msg.path = path
        msg.check_for_collisions = False
        send_goal_future = self.smooth_path_client.send_goal_async(msg)
        final_result_future = self._wrap_send_goal_future(send_goal_future)
        return final_result_future

    def clear_local_costmap(self) -> Future:
        req = ClearEntireCostmap.Request()
        future = self.srv_clear_local_costmap.call_async(req)
        return future

    def clear_global_costmap(self) -> Future:
        req = ClearEntireCostmap.Request()
        future = self.srv_clear_global_costmap.call_async(req)
        return future

    def update_robot_footprint_for_grab(self, obs_marker_id: str):
        if self.omniscient_obstacle_perception:
            # TODO
            return

        obstacle_polygon = self.state.movable_obstacle_tracker.get_obstacle_polygon(
            obs_marker_id
        )
        if obstacle_polygon is None:
            raise Exception("Obstacle not found in namosim world")
        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            raise Exception("Failed to lookup robot pose")
        robot_pose = utils.entity_pose_to_pose2d(robot_pose.pose)
        robot_polygon: geom.Polygon = geom.Point(robot_pose.x, robot_pose.y).buffer(self.robot_radius)  # type: ignore
        footprint: t.Any = geom.MultiPolygon(
            [robot_polygon, obstacle_polygon]
        ).convex_hull

        footprint = affinity.translate(
            footprint, xoff=-robot_pose.x, yoff=-robot_pose.y
        )
        footprint = affinity.rotate(
            footprint,
            angle=robot_pose.degrees,
            origin=(0, 0),  # type: ignore
            use_radians=False,
        )
        footprint = utils.shapely_to_ros_polygon(footprint)

        self.local_footprint_publisher.publish(footprint)
        self.global_footprint_publisher.publish(footprint)

    def update_robot_footprint_for_release(self):
        robot_polygon: geom.Polygon = geom.Point(0, 0).buffer(self.robot_radius)  # type: ignore
        footprint = utils.shapely_to_ros_polygon(robot_polygon)
        self.local_footprint_publisher.publish(footprint)
        self.global_footprint_publisher.publish(footprint)

    def publish_status_marker(self, status: str):
        # Publish marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = -2.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 1.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.0
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.text = f"Status: {status}"
        self.pub_status.publish(marker)
