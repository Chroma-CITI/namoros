import copy
import typing as t
import py_trees
from kobuki_ros_interfaces.msg import Sound
from namoros.behaviors.IgnoreObstacleSync import IgnoreObstacleSync
from namoros.behaviors.ManualSyncPlanner import ManualSyncPlanner
from namoros.behaviors.Backup import BackUp
from namoros.behaviors.Release import Release
from namoros.behaviors.SynchronizePlannerPeriodic import SynchronizePlannerPeriodic
from namoros.behaviors.UnignoreObstacleSync import UnignoreObstacleSync
from namoros_msgs.msg._namo_path import NamoPath
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence
from namoros.behavior_node import NamoBehaviorNode
from namoros.behaviors.Approach import Approach
from namoros.behaviors.ClearGlobalCostmap import ClearGlobalCostmap
from namoros.behaviors.ClearLocalCostmap import ClearLocalCostmap
from namoros.behaviors.DetectConflictsGuard import DetectConflictsGuard
from namoros.behaviors.FaceObstacle import FaceObstacle
from namoros.behaviors.FollowPath import FollowPath
from namoros.behaviors.Pause import Pause
from namoros.behaviors.PlaySound import PlaySound
from namoros.behaviors.RedetectObstacle import RedetectObstacle
from namoros.behaviors.TriggerReplan import TriggerReplan
from namoros.behaviors.UnignoreAllObstacles import UnignoreAllObstacles
from py_trees_ros.trees import BehaviourTree


class ExecutePlan(py_trees.behaviour.Behaviour):
    def __init__(self, node: NamoBehaviorNode):
        super().__init__(name="ExecutePlan")
        self.node = node
        self.tree: BehaviourTree | None = None

    def create_conflict_handling_subtree(self, node: NamoBehaviorNode) -> Behaviour:
        handle_conflicts_root = Selector(
            name="handle_conflicts_root",
            memory=True,
            children=[
                DetectConflictsGuard(node=node),
                Sequence(
                    name="handle_conflict_seq",
                    memory=True,
                    children=[
                        Release(node=self.node),
                        UnignoreAllObstacles(node=node),
                        ManualSyncPlanner(node=node),
                        TriggerReplan(node=node, update_plan=True),
                    ],
                ),
            ],
        )
        return handle_conflicts_root

    def create_postpone_tree(self, seconds: float) -> Behaviour:
        pause_and_sync = py_trees.composites.Parallel(
            name="pause_and_sync",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
            children=[
                Pause(seconds=seconds, node=self.node),
                SynchronizePlannerPeriodic(node=self.node),
            ],
        )
        postpone_root = Sequence(
            name="postpone_root",
            memory=True,
            children=[
                pause_and_sync,
                TriggerReplan(node=self.node, update_plan=True),
            ],
        )
        return postpone_root

    def create_grab_tree(self, path_index: int, path: NamoPath):
        if not self.node.state.plan:
            raise Exception("No plan")
        if not path.is_transfer:
            raise Exception("Not a transfer path")
        obstacle_id: str = path.obstacle_id  # type: ignore
        root = py_trees.composites.Sequence(
            name="grab_seq",
            memory=True,
            children=[
                IgnoreObstacleSync(node=self.node, obstacle_id=obstacle_id),
                PlaySound(node=self.node, sound=Sound.CLEANINGSTART),
                ClearGlobalCostmap(node=self.node),
                ClearLocalCostmap(node=self.node),
                FaceObstacle(node=self.node, obstacle_id=obstacle_id),
                ManualSyncPlanner(
                    node=self.node, path_index=path_index, action_index=0
                ),
                Approach(node=self.node, obstacle_id=obstacle_id),
                ManualSyncPlanner(
                    node=self.node, path_index=path_index, action_index=1
                ),
            ],
        )
        return root

    def create_release_tree(self, path: NamoPath, path_index: int):
        if not self.node.state.plan:
            raise Exception("No plan")
        obstacle_id: str = path.obstacle_id  # type: ignore

        root = py_trees.composites.Sequence(
            name="release",
            memory=True,
            children=[
                PlaySound(node=self.node, sound=Sound.CLEANINGSTART),
                Release(node=self.node),
                BackUp(node=self.node, distance=self.node.agent.grab_start_distance),
                ManualSyncPlanner(
                    node=self.node, path_index=path_index, action_index=0
                ),
                UnignoreObstacleSync(node=self.node, obstacle_id=obstacle_id),
                Pause(node=self.node, seconds=2.0),
            ],
        )

        if not self.node.omniscient_obstacle_perception:
            root.add_child(RedetectObstacle(node=self.node, obstacle_id=obstacle_id))
        root.add_child(TriggerReplan(node=self.node))
        return root

    def create_sub_tree(self):
        if not self.node.state.plan:
            raise Exception("No plan")

        root = py_trees.composites.Sequence(name="execute_plan", memory=True)

        if self.node.state.plan.postpone_steps > 0:
            root.add_child(
                self.create_postpone_tree(
                    seconds=self.node.state.plan.postpone_steps * 0.5
                )
            )
            return BehaviourTree(root)

        paths: t.List[NamoPath] = self.node.state.plan.paths  # type: ignore

        for path_index, namo_path in enumerate(paths):
            nav_path = copy.deepcopy(namo_path.path)
            if namo_path.is_transfer:
                assert namo_path.obstacle_id
                grab = self.create_grab_tree(path_index, namo_path)
                root.add_child(grab)
                # remove the pre-grab and post-release poses since those are executed manually
                nav_path.poses = nav_path.poses[1:-1]  # type: ignore
            else:
                root.add_child(
                    Release(node=self.node)
                )  # Add a release action at the start of each transit path, to make sure robot has released obstacle

            follow_path = FollowPath(
                node=self.node,
                namo_path=namo_path,
                is_evasion=namo_path.is_evasion,
            )

            follow_path_and_sync_planner = py_trees.composites.Parallel(
                name="follow_path_and_sync_planner",
                policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
                children=[follow_path, SynchronizePlannerPeriodic(node=self.node)],
            )

            follow_path_seq = Sequence(
                "follow_path_seq",
                memory=False,
                children=[
                    self.create_conflict_handling_subtree(self.node),
                    follow_path_and_sync_planner,
                ],
            )

            root.add_children([follow_path_seq])

            if namo_path.is_evasion:
                root.add_child(TriggerReplan(node=self.node))
            if namo_path.is_transfer:
                release = self.create_release_tree(
                    path_index=path_index + 1, path=namo_path
                )
                root.add_child(release)

        return BehaviourTree(root, unicode_tree_debug=False)

    def initialise(self):
        if not self.node.state.plan:
            raise Exception("No plan")
        self.tree = self.create_sub_tree()
        snapshot_visitor = py_trees.visitors.DebugVisitor()
        self.tree.visitors.append(snapshot_visitor)
        # py_trees.display.render_dot_tree(
        #     self.tree.root, name="execute_plan_tree", target_directory="."
        # )

    def update(self):
        if not self.tree:
            raise Exception("No tree")
        self.tree.tick()
        self.status = self.tree.root.status
        return self.status
