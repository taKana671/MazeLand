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

    def __init__(self, model, row, col, pos):
        super().__init__(BulletRigidBodyNode(f'wall_{row}_{col}'))
        self.row = row
        self.col = col

        self.set_pos(pos)
        self.set_scale(Vec3(2, 2, 4))
        self.set_collide_mask(BitMask32.bit(1))

        self.model = model.copy_to(self)
        end, tip = self.model.get_tight_bounds()
        self.node().add_shape(BulletBoxShape((tip - end) / 2))
        self.node().set_mass(0)


class MazeBuilder3D:

    def __init__(self, world):
        # self.size = Size(21, 21)  # rows, cols
        # self.grid = create_maze(self.size.rows, self.size.cols)

        # self.gen = self.get_wall()
        self.block = Cube()
        self.world = world
        self.walls = NodePath('walls')
        self.walls.reparent_to(base.render)

        # self.create_grid()

    def build(self, rows, cols):
        self.size = Size(rows, cols)
        self.grid = create_maze(*self.size)

        for r in range(self.size.rows):
            for c in range(self.size.cols):
                if self.grid[r, c] == Maze.WALL:
                    pos = Point3((c - self.size.cols // 2) * 2, (-r + self.size.rows // 2) * 2, 1)
                    wall = Wall(self.block, r, c, pos)
                    wall.set_color(LColor(1, 0, 0, 1))
                    wall.reparent_to(self.walls)
                    self.world.attach(wall.node())

    
 
    # def update(self, dt):
    #     try:
    #         # for elem in self.queue:

    #         #     wall = self.grid[elem]
    #         #     wall.go_up(dt)
    #         #         # self.queue.popleft()

    #         # (r, c) = self.queue.popleft()
    #         # wall = self.grid[(r, c)]
    #         # if not wall.go_up(dt):
    #         #     self.queue.append((r, c))
    #         x, y = next(self.gen)
    #         # pos = Point3(x * 2 - 10, y * -2 + 10, 1)  # できる
    #         pos = Point3((x - self.size.cols // 2) * 2, (-y + self.size.rows // 2) * 2, 1)
    #         wall = Wall(self.block, y, x, pos)
    #         # wall = Wall('wall', self.block, pos, Vec3(0, 0, 0), Vec3(2), BitMask32(1))
    #         wall.set_color(LColor(1, 0, 0, 1))
    #         wall.reparent_to(self.walls)
    #         self.world.attach(wall.node())
    #     except StopIteration:
    #     # except IndexError:
    #         wall = Wall(self.block, 0, 0, Point3(0, 0, 2))
    #         # wall = Wall('wall', self.block, Point3(0, 0, 2), Vec3(0, 0, 0), Vec3(2), BitMask32(1))
    #         wall.set_color(LColor(1, 1, 0, 1))
    #         wall.reparent_to(self.walls)
    #         self.world.attach(wall.node())
    #         pass