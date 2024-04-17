import numpy as np
import random

from enum import Enum


class Maze:

    WALL = 1
    AISLE = 0
    EXTENDING = 2


class Directions2D(Enum):

    BOTTOM = (0, 1)
    UPPER = (0, -1)
    RIGHT = (1, 0)
    LEFT = (-1, 0)

    def __init__(self, col, row):
        super().__init__()
        self.col = col
        self.row = row


def starts_pts(rows, cols):
    li = [(x, y) for y in range(1, rows - 1) for x in range(1, cols - 1) if y % 2 == 0 and x % 2 == 0]
    random.shuffle(li)
    return li


def is_extendable(grid, direction, x, y):
    ahead_2x = x + direction.col * 2
    ahead_2y = y + direction.row * 2

    if grid[ahead_2y, ahead_2x] != Maze.EXTENDING:
        return True


def extend_wall(grid, org_x, org_y):
    x, y = org_x, org_y

    while True:

        match grid[y, x]:
            case Maze.AISLE:
                grid[y, x] = Maze.EXTENDING
            case Maze.WALL:
                grid[grid == Maze.EXTENDING] = Maze.WALL
                return

        if not (directions := [d for d in Directions2D if is_extendable(grid, d, x, y)]):
            grid[grid == Maze.EXTENDING] = Maze.AISLE
            x, y = org_x, org_y
            continue

        direction = random.choice(directions)
        grid[y + direction.row, x + direction.col] = Maze.EXTENDING
        x += direction.col * 2
        y += direction.row * 2


def create_grid(rows, cols):
    grid = np.zeros((rows, cols))
    grid[0, :] = Maze.WALL
    grid[-1, :] = Maze.WALL
    grid[:, 0] = Maze.WALL
    grid[:, -1] = Maze.WALL

    return grid


def create_maze(rows, cols):
    grid = create_grid(rows, cols)
    starts = starts_pts(rows, cols)
    # print(starts)

    for x, y in starts:
        if grid[y, x] != Maze.WALL:
            extend_wall(grid, x, y)

    print(grid)
    return grid


if __name__ == '__main__':
    create_maze(9, 11) 