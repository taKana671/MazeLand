import random
import types
from enum import Enum
from typing import NamedTuple
from collections import deque
import numpy as np
from panda3d.bullet import BulletRigidBodyNode, BulletBoxShape
from panda3d.core import NodePath, PandaNode
from panda3d.core import Vec3, Point3, LVecBase2i, LColor, BitMask32, LPoint2i, Point2

# from create_maze_2d import create_maze
from create_geomnode import Cube


class Wall(NodePath):

    def __init__(self, model, suffix, pos, scale=Vec3(2, 2, 4)):
        super().__init__(BulletRigidBodyNode(f'wall_{suffix}'))
        self.set_pos(pos)
        self.set_scale(Vec3(2, 2, 4))
        self.set_collide_mask(BitMask32.bit(1) | BitMask32.bit(2))

        self.model = model.copy_to(self)
        end, tip = self.model.get_tight_bounds()
        self.node().add_shape(BulletBoxShape((tip - end) / 2))
        self.node().set_mass(0)


class Space(NamedTuple):

    row: int
    col: int


class Direction2D(Enum):

    BACKWARD = (0, 1)
    FORWARD = (0, -1)
    RIGHTWARD = (1, 0)
    LEFTWARD = (-1, 0)

    def __init__(self, col, row):
        super().__init__()
        self.col = col
        self.row = row

    @classmethod
    def get_direction(self, col, row):
        for mem in self:
            if mem.value == (col, row):
                return mem


class Grid:

    def __init__(self, rows, cols, side_length=2):
        self.rows = rows if rows % 2 != 0 else rows - 1
        self.cols = cols if cols % 2 != 0 else cols - 1
        self.side_length = side_length

    @property
    def entrance(self):
        return Space(self.rows - 1, self.cols - 2)

    @property
    def exit(self):
        return Space(0, 1)

    # def move(self, current_space, direction, move_space):
    #     if (next_row := direction.row * move_space + current_space.row) < 0:
    #         next_row = 0
    #     elif next_row >= self.rows:
    #         next_row = self.rows - 1

    #     if next_col := direction.col * move_space + current_space.x < 0:
    #         next_col = 0
    #     elif next_col >= self.cols:
    #         next_col = self.cols - 1

    #     return Space(next_row, next_col)

    def space_to_world(self, row, col):
        x = (col - self.cols // 2) * 2
        y = (-row + self.rows // 2) * 2
        return Point2(x, y)

    def forward(self, current_space, space_cnt=1):
        if (next_row := Direction2D.FORWARD.row * space_cnt + current_space.row) < 0:
            next_row = 0

        return Space(next_row, current_space.col)

    def backward(self, current_space, space_cnt=1):
        if (next_row := Direction2D.BACKWARD.row * space_cnt + current_space.row) >= self.rows:
            next_row = self.rows - 1

        return Space(next_row, current_space.col)

    def leftward(self, current_space, space_cnt=1):
        if (next_col := Direction2D.LEFTWARD.col * space_cnt + current_space.col) < 0:
            next_col = 0

        return Space(current_space.row, next_col)

    def rightward(self, current_space, space_cnt=1):
        if (next_col := Direction2D.RIGHTWARD.col * space_cnt + current_space.col) >= self.cols:
            next_col = self.cols - 1

        return Space(current_space.row, next_col)


maze = types.SimpleNamespace(wall=1, aisle=0, extending=2)


class MazeBuilder3D:

    WALL = 1
    AISLE = 0
    EXTENDING = 2

    def __init__(self, world, grid):
        self.block = Cube()
        self.world = world
        self.grid = grid

        self.walls = NodePath('walls')
        self.walls.reparent_to(base.render)

    def build(self):
        maze = self.create_maze(self.grid.rows, self.grid.cols)
        # maze[*self.grid.entrance] = self.AISLE 
        maze[*self.grid.exit] = self.AISLE
        enter_space = self.grid.entrance
        # exit_space = self.grid.exit

        for r in range(self.grid.rows):
            for c in range(self.grid.cols):
                if maze[r, c] == self.WALL:
                    p2 = self.grid.space_to_world(r, c)
                    wall = Wall(self.block, f'{r}_{c}', Point3(p2, 2))
                    wall.set_color(LColor(1, 0, 0, 1))
                    wall.reparent_to(self.walls)
                    self.world.attach(wall.node())

                    if (r, c) == enter_space:  # or (r, c) == exit_space:
                        wall.hide()

    def create_maze(self, rows, cols):
        maze = np.zeros((rows, cols))
        maze[[0, -1]] = self.WALL
        maze[:, [0, -1]] = self.WALL

        starts = [(x, y) for y in range(1, rows - 1) for x in range(1, cols - 1) if y % 2 == 0 and x % 2 == 0]
        random.shuffle(starts)

        for x, y in starts:
            if maze[y, x] != self.WALL:
                self.extend_wall(maze, x, y)

        print(maze)
        return maze

    def is_extendable(self, maze, direction, x, y):
        ahead_2x = x + direction.col * 2
        ahead_2y = y + direction.row * 2

        if maze[ahead_2y, ahead_2x] != self.EXTENDING:
            return True

    def extend_wall(self, maze, org_x, org_y):
        x, y = org_x, org_y

        while True:

            match maze[y, x]:
                case self.AISLE:
                    maze[y, x] = self.EXTENDING
                case self.WALL:
                    maze[maze == self.EXTENDING] = self.WALL
                    return

            if not (directions := [d for d in Direction2D if self.is_extendable(maze, d, x, y)]):
                maze[maze == self.EXTENDING] = self.AISLE
                x, y = org_x, org_y
                continue

            direction = random.choice(directions)
            maze[y + direction.row, x + direction.col] = self.EXTENDING
            x += direction.col * 2
            y += direction.row * 2