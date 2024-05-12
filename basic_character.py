from enum import Enum, auto
from panda3d.core import NodePath, PandaNode, BitMask32


class Direction(Enum):

    FORWARD = auto()
    BACKWARD = auto()
    LEFTWARD = auto()
    RIGHTWARD = auto()
    UPWARD = auto()


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


class Sensor(NodePath):

    def __init__(self, direction, pos, world):
        super().__init__(PandaNode(direction.name))
        self.world = world
        self.direction = direction
        self.set_pos(pos)

    def detect_obstacles(self, pos_from, bit=2):
        pos_to = self.get_pos(base.render)

        if (result := self.world.ray_test_closest(
                pos_from, pos_to, mask=BitMask32.bit(bit))).has_hit():
            return result