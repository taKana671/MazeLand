import random
from enum import Enum
from typing import NamedTuple
from collections import deque
import numpy as np
from panda3d.bullet import BulletRigidBodyNode, BulletBoxShape
from panda3d.core import NodePath, PandaNode
from panda3d.core import Vec3, Point3, LVecBase2i, LColor, BitMask32

from create_maze_2d import create_maze, Maze
from create_geomnode import Cube


class Size(NamedTuple):

    rows: int
    cols: int


class Wall(NodePath):

    def __init__(self, model, suffix, pos):
        super().__init__(BulletRigidBodyNode(f'wall_{suffix}'))
        self.set_pos(pos)
        self.set_scale(Vec3(2, 2, 4))
        self.set_collide_mask(BitMask32.bit(1) | BitMask32.bit(2))

        self.model = model.copy_to(self)
        end, tip = self.model.get_tight_bounds()
        self.node().add_shape(BulletBoxShape((tip - end) / 2))
        self.node().set_mass(0)


class MazeBuilder3D:

    def __init__(self, world):
        self.block = Cube()
        self.world = world
        self.walls = NodePath('walls')
        self.walls.reparent_to(base.render)

    def build(self, rows, cols):
        if rows % 2 == 0 or cols % 2 == 0:
            raise ValueError('rows and cols must be odd numbers.')

        self.size = Size(rows, cols)
        self.grid = create_maze(*self.size)

        self.grid[0, 1] = Maze.AISLE
        self.grid[self.size.rows - 1, self.size.cols - 2] = Maze.AISLE

        for r in range(self.size.rows):
            for c in range(self.size.cols):
                if self.grid[r, c] == Maze.WALL:
                    pos = Point3((c - self.size.cols // 2) * 2, (-r + self.size.rows // 2) * 2, 2)
                    wall = Wall(self.block, f'{r}_{c}', pos)
                    wall.set_color(LColor(1, 0, 0, 1))
                    wall.reparent_to(self.walls)
                    self.world.attach(wall.node())