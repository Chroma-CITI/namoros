import typing as t
from namoros_msgs.msg import NamoEntity
from namoros.utils import Pose2D
from geometry_msgs.msg import Pose


class WorldStateTracker:
    """
    Tracks poses of robots and movable obstacles. Used to update planner state with current observed state of the world.
    """

    def __init__(self):
        self.robots: t.Dict[str, NamoEntity] = {}
        self.obstacles: t.Dict[str, Pose] = {}
        self.robot_to_obstacle: t.Dict[str, str] = {}  # held obstacle

    def update_robot(self, robot: NamoEntity):
        self.robots[robot.entity_id] = robot

    def update_obstacle(self, entity_id: str, pose: Pose):
        # entity = NamoEntity()
        # entity.entity_id = entity_id
        # entity.pose.x = pose.position.x
        # entity.pose.y = pose.position.y
        # entity.pose.z = pose.position.z
        # entity.pose.angle_degrees = pose.degrees
        self.obstacles[entity_id] = pose

    def grab_obstacle(self, robot_id: str, obstacle_id: str):
        self.robot_to_obstacle[robot_id] = obstacle_id

    def release_obstacle(self, robot_id: str):
        if robot_id in self.robot_to_obstacle:
            del self.robot_to_obstacle[robot_id]

    def clear(self):
        self.robots = {}
        self.obstacles = {}
