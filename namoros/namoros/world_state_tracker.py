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

    def update_robot(self, robot: NamoEntity):
        self.robots[robot.entity_id] = robot

    def update_obstacle(self, entity_id: str, pose: Pose2D):
        entity = NamoEntity()
        entity.entity_id = entity_id
        entity.pose.x = pose.x
        entity.pose.y = pose.y
        entity.pose.angle_degrees = pose.degrees
        self.obstacles[entity_id] = entity
