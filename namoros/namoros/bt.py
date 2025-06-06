import py_trees
import rclpy
from kobuki_ros_interfaces.msg import Sound
from py_trees.behaviour import Behaviour
from py_trees.common import ParallelPolicy, Status
from py_trees.composites import Parallel, Selector, Sequence
from py_trees_ros.trees import BehaviourTree
from rclpy.executors import MultiThreadedExecutor

from namoros.behavior_node import NamoBehaviorNode
from namoros.behaviors.ComputePlan import ComputePlan
from namoros.behaviors.ClearNewMovables import ClearNewMovables
from namoros.behaviors.ExecutePlan import ExecutePlan
from namoros.behaviors.InterruptRobot import InterruptRobot
from namoros.behaviors.ManualSyncPlanner import ManualSyncPlanner
from namoros.behaviors.NewObstacleGuard import NewObstacleGuard
from namoros.behaviors.PlaySound import PlaySound
from namoros.behaviors.Release import Release
from namoros.behaviors.ReplanGuard import ReplanGuard
from namoros.behaviors.SynchronizePlannerPeriodic import SynchronizePlannerPeriodic
from namoros.behaviors.UnignoreAllObstacles import UnignoreAllObstacles
from namoros.behaviors.UpdatePlanGuard import UpdatePlanGuard
from namoros.behaviors.UpdatePlan import UpdatePlan

from namoros.behaviors.WaitForFullObstacleDetection import (
    WaitForFullObstacleDetection,
)
from namoros.behaviors.WaitForGoalPose import WaitForGoalPose
from namoros.behaviors.WaitForInitPose import WaitForInitPose


def create_namo_tree(node: NamoBehaviorNode) -> Behaviour:
    new_movable_guard = NewObstacleGuard(node=node)
    new_movable_seq = Sequence(
        name="new_movable_seq",
        memory=True,
        children=[
            InterruptRobot(node=node),
            WaitForFullObstacleDetection(node=node),
            ClearNewMovables(node=node),
        ],
    )
    handle_new_movable = Selector(
        name="new_movable_root",
        memory=False,
        children=[new_movable_guard, new_movable_seq],
    )

    update_plan_seq = Sequence(
        name="update_plan_seq",
        memory=True,
        children=[InterruptRobot(node=node), UpdatePlan(node=node)],
    )
    update_plan_guard = Selector(
        name="update_plan_guard",
        memory=True,
        children=[UpdatePlanGuard(node=node), update_plan_seq],
    )
    excute_plan_root = Sequence(
        name="excute_plan_root",
        memory=False,
        children=[
            update_plan_guard,
            PlaySound(node=node, sound=Sound.CLEANINGSTART),
            ExecutePlan(node=node),
            PlaySound(node=node, sound=Sound.CLEANINGEND),
        ],
    )
    compute_and_execute_plan = Sequence(
        name="compute_and_execute_plan",
        memory=True,
        children=[
            InterruptRobot(node=node),
            Release(node=node),
            UnignoreAllObstacles(node=node),
            ManualSyncPlanner(node=node),
            ComputePlan(node=node),
            excute_plan_root,
        ],
    )
    new_movable_and_main = Sequence(
        name="new_movable_and_main",
        memory=False,
        children=[
            ReplanGuard(node=node),
            handle_new_movable,
            compute_and_execute_plan,
        ],
    )
    root = Sequence(
        name="root",
        memory=True,
        children=[
            PlaySound(node=node, sound=Sound.ON),
            WaitForInitPose(node=node),
            PlaySound(node=node, sound=Sound.BUTTON),
            WaitForGoalPose(node=node),
            PlaySound(node=node, sound=Sound.BUTTON),
            new_movable_and_main,
        ],
    )
    return root


def main(args=None):
    rclpy.init(args=args)
    py_trees.logging.level = py_trees.logging.Level.INFO
    executor = MultiThreadedExecutor()
    node = NamoBehaviorNode()
    executor.add_node(node)
    root = create_namo_tree(node=node)
    tree = BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup(node=node)
    py_trees.display.render_dot_tree(
        root,
        name="behavior_tree",
        target_directory=".",
        collapse_decorators=True,
        visibility_level=py_trees.common.VisibilityLevel.COMPONENT,
    )
    snapshot_visitor = py_trees.visitors.DebugVisitor()

    def post_tick_handler(tree: BehaviourTree):
        if tree.root.status == Status.SUCCESS:
            node.get_logger().info("Behavior tree finished successfully.")
            node.reset()
        elif tree.root.status == Status.FAILURE:
            node.get_logger().error("Behavior tree failed.")
            tree.shutdown()

    tree.add_post_tick_handler(post_tick_handler)  # type: ignore
    tree.visitors.append(snapshot_visitor)

    tree.tick_tock(period_ms=500)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()


if __name__ == "__main__":
    main()
