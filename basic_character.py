from enum import Enum, auto

from panda3d.core import NodePath, PandaNode
from panda3d.core import BitMask32, Vec3


class Status(Enum):

    MOVE = auto()
    STOP = auto()
    LEFT_TURN = auto()
    RIGHT_TURN = auto()
    U_TURN = auto()
    CHECK_ROUTE = auto()
    GO_BACK = auto()
    GO_FORWARD = auto()
    DO_JUMP = auto()


class Direction(Enum):

    FORWARD = (0, 1, 0)
    BACKWARD = (0, -1, 0)
    LEFTWARD = (-1, 0, 0)
    RIGHTWARD = (1, 0, 0)
    UPWARD = (0, 0, 1)
    DOWNWARD = (0, 0, -1)

    def get_vector(self, orientation=1):
        return Vec3(self.value) * orientation

    def get_direction(self, orientation=1):
        match self.value:
            case (x, y, z) if x != 0:
                return x * orientation
            case (x, y, z) if y != 0:
                return y * orientation
            case (x, y, z) if z != 0:
                return z * orientation


class Sensor(NodePath):
    """Detect obstacles by casting ray.
       Args:
            world (panda3d.bullet.BulletWorld)
            direction (Direction)
            orient (float) 1: starts from entrance, -1: starts from exit
            dist (float)
    """

    def __init__(self, world, direction, orient=1.0, dist=1.5):
        super().__init__(PandaNode(direction.name))
        self.world = world
        self.direction = direction
        pos = direction.get_vector(orient) * dist
        self.set_pos(pos)

    def detect_obstacles(self, pos_from, bit=2):
        pos_to = self.get_pos(base.render)

        if (result := self.world.ray_test_closest(
                pos_from, pos_to, mask=BitMask32.bit(bit))).has_hit():
            return result