import typing as t
from namoros_msgs.msg import NamoEntity
from namoros.utils import Pose2D


class WorldStateTracker:
    """
    Tracks poses of robots and movable obstacles. Used to update planner state with current observed state of the world.
    """

    def __init__(self):
        self.robots: t.Dict[str, NamoEntity] = {}
        self.obstacles: t.Dict[str, NamoEntity] = {}
        self._ignored_obstacles: t.Set[str] = set()

    def update_robot(self, robot: NamoEntity):
        self.robots[robot.entity_id] = robot

    def update_obstacle(self, entity_id: str, pose: Pose2D):
        if entity_id in self._ignored_obstacles:
            return
        entity = NamoEntity()
        entity.entity_id = entity_id
        entity.pose.x = pose.x
        entity.pose.y = pose.y
        entity.pose.angle_degrees = pose.degrees
        self.obstacles[entity_id] = entity

    def ignore_obstacle(self, obstacle_id: str):
        self._ignored_obstacles.add(obstacle_id)
        if obstacle_id in self.obstacles:
            del self.obstacles[obstacle_id]

    def unignore_obstacle(self, obstacle_id: str):
        self._ignored_obstacles.remove(obstacle_id)

    def clear(self):
        self.robots = {}
        self.obstacles = {}
        self._ignored_obstacles = {}
