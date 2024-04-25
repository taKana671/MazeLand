import random
import types
from enum import Enum
from typing import NamedTuple
from collections import deque
import numpy as np
from panda3d.bullet import BulletRigidBodyNode, BulletBoxShape
from panda3d.core import NodePath, PandaNode
from panda3d.core import Vec3, Point3, LVecBase2i, LColor, BitMask32, LPoint2i, Point2

from create_maze_2d import WallExtendingAlgorithm
from create_geomnode import Cube


class Wall(NodePath):

    def __init__(self, name, pos, scale, mask):
        super().__init__(BulletRigidBodyNode(f'wall_{name}'))
        cube = Cube()
        self.model = self.attach_new_node(cube.node())

        self.set_pos(pos)
        self.set_scale(scale)
        self.set_collide_mask(mask)

        end, tip = self.model.get_tight_bounds()
        self.node().add_shape(BulletBoxShape((tip - end) / 2))
        self.node().set_mass(0)


class Space(NamedTuple):

    row: int
    col: int


class MazeBuilder:

    def __init__(self, world, rows, cols, side=2):
        self.world = world
        self.wall_size = Vec3(side, side, side * 2)
        self.rows = rows if rows % 2 != 0 else rows - 1
        self.cols = cols if cols % 2 != 0 else cols - 1

        self.entrance = Space(self.rows - 1, self.cols - 2)
        self.exit = Space(0, 1)

        self.walls = NodePath('walls')
        self.walls.reparent_to(base.render)

    def get_entrance(self):
        return self.space_to_cartesian(*self.entrance)

    def space_to_cartesian(self, row, col):
        x = (col - self.cols // 2) * self.wall_size.x
        y = (-row + self.rows // 2) * self.wall_size.y
        return Point2(x, y)

    def build(self):
        # config = types.SimpleNamespace(wall=1, aisle=0, extending=2)
        grid = WallExtendingAlgorithm(self.rows, self.cols).create_maze()
        wall_z = self.wall_size.z / 2

        for r in range(self.rows):
            for c in range(self.cols):
                if grid[r, c] == WallExtendingAlgorithm.WALL:
                    xy = self.space_to_cartesian(r, c)
                    pos = Point3(xy, wall_z)

                    match (r, c):
                        case self.entrance:
                            mask = BitMask32.bit(1) | BitMask32.bit(2)
                            hide = True
                        case self.exit:
                            mask = BitMask32.bit(1) | BitMask32.bit(3)
                            hide = True
                        case _:
                            mask = BitMask32.bit(1) | BitMask32.bit(2)
                            hide = False

                    self.make_wall(f'{r}_{c}', pos, self.wall_size, mask, hide)

    def make_wall(self, name, pos, scale, mask, hide=False):
        wall = Wall(name, pos, scale, mask)
        wall.set_color(LColor(1, 0, 0, 1))
        wall.reparent_to(self.walls)
        self.world.attach(wall.node())

        if hide:
            wall.hide()
