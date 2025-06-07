import traceback
import typing as t
from namoros.namo_planner import NamoPlanner
from namoros.utils import Pose2D, entity_pose_to_pose2d
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter as RosParam
from std_msgs.msg import Header
import namoros.utils as utils
from nav_msgs.msg import OccupancyGrid
from namoros_msgs.msg import NamoAction, NamoConflict
from namoros_msgs.srv import (
    AddOrUpdateMovableObstacle,
    SimulatePath,
    GetEntityPolygon,
    SynchronizeState,
    DetectConflicts,
    EndPostpone,
)
from namoros_msgs.action import ComputePlan, UpdatePlan
from rclpy.executors import MultiThreadedExecutor
from namosim.navigation.basic_actions import (
    Wait,
    Translation,
    Rotation,
    Advance,
    Grab,
    Release,
    Action,
)
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from namosim.navigation.navigation_plan import Plan
from namosim.navigation.conflict import ConflictType
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from namosim.navigation.navigation_path import TransferPath
from namoros_msgs.msg import NamoEntity


class PlannerNode(Node):
    def __init__(self):
        super().__init__("namo_planner", parameter_overrides=[])
        self.declare_parameters(
            namespace="",
            parameters=[
                ("scenario_file", RosParam.Type.STRING),
                ("agent_id", RosParam.Type.STRING),
                ("omniscient_obstacle_perception", RosParam.Type.BOOL),
            ],
        )
        self.scenario_file = t.cast(str, self.get_parameter("scenario_file").value)
        self.agent_id = t.cast(str, self.get_parameter("agent_id").value)
        self.omniscient_obstacle_perception = t.cast(
            bool, self.get_parameter("omniscient_obstacle_perception").value
        )
        self.namo_planner = NamoPlanner(
            ros_node=self,
            scenario_file=self.scenario_file,
            agent_id=self.agent_id,
            omniscient_obstacle_perception=self.omniscient_obstacle_perception,
            logger=self.get_logger(),
        )
        self.namo_planner.publish_world()
        self.services_cb_group = ReentrantCallbackGroup()
        self.sub_map = self.create_subscription(
            OccupancyGrid, "map", self.map_callback, 1
        )
        self.srv_add_obstacle = self.create_service(
            AddOrUpdateMovableObstacle,
            "namo_planner/add_or_update_movable_obstacle",
            self.add_or_update_obstacle_callback,
            callback_group=self.services_cb_group,
        )

        self.srv_simulate_path = self.create_service(
            SimulatePath,
            "namo_planner/simulate_path",
            self.simulate_path_callback,
            callback_group=self.services_cb_group,
        )
        self.srv_get_entity_polygon = self.create_service(
            GetEntityPolygon,
            "namo_planner/get_entity_polygon",
            self.get_entity_polygon_callback,
            callback_group=self.services_cb_group,
        )

        self.sync_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv_synchronize_state = self.create_service(
            SynchronizeState,
            "namo_planner/synchronize_state",
            self.synchronize_state,
            callback_group=self.sync_cb_group,
        )
        self.srv_detect_conflicts = self.create_service(
            DetectConflicts,
            "namo_planner/detect_conflicts",
            self.detect_conflicts,
            callback_group=self.sync_cb_group,
        )
        self.srv_end_postpone = self.create_service(
            EndPostpone,
            "namo_planner/end_postpone",
            self.end_postpone,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # actions
        compute_plan_cb_group = MutuallyExclusiveCallbackGroup()
        self.action_compute_plan = ActionServer(
            node=self,
            action_type=ComputePlan,
            action_name=f"namo_planner/compute_plan",
            execute_callback=self.compute_plan_callback,
            callback_group=compute_plan_cb_group,
        )
        self.action_update_plan = ActionServer(
            node=self,
            action_type=UpdatePlan,
            action_name="namo_planner/update_plan",
            execute_callback=self.update_plan_callback,
            callback_group=compute_plan_cb_group,
        )

        # state
        self.current_plan: Plan | None = None
        self.is_planning: bool = False

    def map_callback(self, data: OccupancyGrid):
        pass

    def add_or_update_obstacle_callback(
        self,
        req: AddOrUpdateMovableObstacle.Request,
        res: AddOrUpdateMovableObstacle.Response,
    ):
        polygon = utils.ros_polygon_to_shapely_polygon(req.polygon)
        pose = Pose2D(req.pose.x, req.pose.y, req.pose.angle_degrees)
        self.namo_planner.add_movable_obstacle(
            entity_id=req.obstacle_id, pose=pose, polygon=polygon
        )
        self.namo_planner.publish_world()
        res.result = True
        return res

    def compute_plan_callback(self, goal_handle: ServerGoalHandle):
        try:
            self.is_planning = True

            goal: ComputePlan.Goal = goal_handle.request
            result = ComputePlan.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Compute plan canceled")
                return result

            self.get_logger().info("Computing plan")
            robot_pose = entity_pose_to_pose2d(goal.start_pose.pose)
            goal_pose = entity_pose_to_pose2d(goal.goal_pose.pose)
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"
            self.namo_planner.reset_robot_pose(self.agent_id, robot_pose)
            self.namo_planner.reset_goal_pose(goal_pose)
            plan_result = self.namo_planner.compute_plan(header=header)
            if plan_result:
                plan, plan_msg = plan_result
                result.plan = plan_msg
                self.current_plan = plan
                self.get_logger().info(f"Finished computing plan")
            return result
        finally:
            self.is_planning = False

    def update_plan_callback(self, goal_handle: ServerGoalHandle):
        try:
            self.is_planning = True

            result = UpdatePlan.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Update plan canceled")
                return result

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"
            think_result = self.namo_planner.think(header)
            if think_result:
                updated_plan, update_plan_msg = think_result
                self.current_plan = updated_plan
                result.plan = update_plan_msg
            return result
        finally:
            self.is_planning = False

    def end_postpone(self, req: EndPostpone.Request, res: EndPostpone.Response):
        self.namo_planner.end_postpone()
        return res

    def message_to_action(self, msg: NamoAction) -> Action:
        if msg.action_type == NamoAction.WAIT:
            return Wait()
        elif msg.action_type == NamoAction.TRANSLATION:
            return Translation(v=(msg.translation_x, msg.translation_y))
        elif msg.action_type == NamoAction.ROTATION:
            return Rotation(angle=msg.rotation_angle_degrees)
        elif msg.action_type == NamoAction.GRAB:
            return Grab(entity_uid=msg.obstacle_id, distance=msg.distance)
        elif msg.action_type == NamoAction.RELEASE:
            return Release(entity_uid=msg.obstacle_id, distance=msg.distance)
        elif msg.action_type == NamoAction.ADVANCE:
            return Advance(distance=msg.distance)
        else:
            raise Exception("Unsupported action type")

    def simulate_path_callback(
        self, req: SimulatePath.Request, res: SimulatePath.Response
    ):
        for action_msg in req.path.actions:
            action = self.message_to_action(action_msg)
            self.namo_planner.step_simulation({self.agent_id: action})
            # self.namo_planner.publish_world()
        res.result = True
        return res

    def get_entity_polygon_callback(
        self, req: GetEntityPolygon.Request, res: GetEntityPolygon.Response
    ):
        res.polygon = utils.shapely_to_ros_polygon(
            self.namo_planner.world.dynamic_entities[req.entity_id].polygon
        )
        return res

    def synchronize_state(
        self, req: SynchronizeState.Request, res: SynchronizeState.Response
    ) -> SynchronizeState.Response:
        try:
            if self.is_planning:
                return res

            if not self.current_plan:
                self.namo_planner.synchronize_state(
                    req.other_observed_robots, req.observed_obstacles  # type: ignore
                )
                res.result = True
                return res

            if req.path_index >= 0 and req.action_index >= 0:
                self.get_logger().info(
                    f"Setting plan to path {req.path_index}, action {req.action_index}"
                )
                self.current_plan.set_current_action_index(
                    req.path_index, req.action_index
                )

                path = self.current_plan.get_current_path()
                robot_pose = path.robot_path.poses[req.action_index]
                self.namo_planner.reset_robot_pose(
                    self.agent_id,
                    pose=Pose2D(robot_pose[0], robot_pose[1], robot_pose[2]),
                    resolve_collisions=False,
                )

                observed_obstacles: t.List[NamoEntity] = []
                if path.is_transfer:
                    assert isinstance(path, TransferPath)

                    if path.action_index > 0:
                        self.namo_planner.world.entity_to_agent[
                            path.obstacle_uid
                        ] = self.agent_id

                    obstacle_pose = path.obstacle_path.poses[path.action_index]
                    self.namo_planner.reset_obstacle_pose(
                        path.obstacle_uid, obstacle_pose, resolve_collisions=False
                    )

                    for obs in t.cast(t.List[NamoEntity], req.observed_obstacles):
                        if obs.entity_id != path.obstacle_uid:
                            observed_obstacles.append(obs)
                else:
                    self.namo_planner.world.drop_obstacle(
                        self.agent_id, resolve_collisions=False
                    )
                    observed_obstacles = req.observed_obstacles  # type: ignore

                self.namo_planner.synchronize_state(
                    req.other_observed_robots, observed_obstacles
                )
            else:
                path = self.current_plan.get_current_path()
                start_idx = path.action_index
                poses = path.robot_path.poses[start_idx:-1]
                robot_pose = Pose2D(
                    req.observed_robot_pose.x,
                    req.observed_robot_pose.y,
                    req.observed_robot_pose.angle_degrees,
                )
                min_dist = float("inf")
                actions_ahead = 0

                for i, pose in enumerate(poses):
                    d = utils.get_distance2d(
                        Pose2D(pose[0], pose[1], pose[2]), robot_pose
                    )
                    if d < min_dist:
                        min_dist = d
                        actions_ahead = i

                for i in range(actions_ahead):
                    action = self.current_plan.pop_next_action()
                    self.namo_planner.step_simulation({self.agent_id: action})

                self.namo_planner.synchronize_state(
                    req.other_observed_robots, req.observed_obstacles
                )
            res.result = True
            return res
        except Exception as e:
            self.get_logger().error(
                f"An error occurred during synchronize state: {str(e)}"
            )
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            res.result = False
            return res

    def detect_conflicts(
        self, req: DetectConflicts.Request, res: DetectConflicts.Response
    ):
        conflicts = self.namo_planner.detect_conflicts()

        for c in conflicts:
            msg = NamoConflict()
            if c.conflict_type in [
                ConflictType.ROBOT_ROBOT,
                ConflictType.SSA,
                ConflictType.SIMULTAEOUS_GRAB,
                ConflictType.STOLEN_OBSTACLE,
            ]:
                msg.conflict_type = NamoConflict.ROBOT
            else:
                msg.conflict_type = NamoConflict.OBSTACLE

            self.get_logger().info(f"{self.agent_id} detected conflict: {c}")
            msg.conflict_type = NamoConflict.ROBOT
            res.conflicts.append(msg)
        return res


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = PlannerNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()


if __name__ == "__main__":
    main()
