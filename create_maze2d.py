import random
import numpy as np


class WallExtendingAlgorithm:

    WALL = 1
    PASSAGE = 0
    EXTENDING = 2

    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols

    def create_maze(self):
        grid = np.zeros((self.rows, self.cols))
        grid[[0, -1]] = self.WALL
        grid[:, [0, -1]] = self.WALL

        starts = [pt for pt in self.starts_pts()]
        random.shuffle(starts)

        for x, y in starts:
            if grid[y, x] != self.WALL:
                self.extend_wall(grid, x, y)

        # print(grid)
        return grid

    def starts_pts(self):
        for y in range(1, self.rows - 1):
            for x in range(1, self.cols - 1):
                if y % 2 == 0 and x % 2 == 0:
                    yield (x, y)

    def extendable_directions(self, grid, x, y):
        for direction in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            ahead_2x = x + direction[0] * 2
            ahead_2y = y + direction[1] * 2

            if grid[ahead_2y, ahead_2x] != self.EXTENDING:
                yield direction

    def extend_wall(self, grid, org_x, org_y):
        x, y = org_x, org_y

        while True:

            match grid[y, x]:
                case self.PASSAGE:
                    grid[y, x] = self.EXTENDING
                case self.WALL:
                    grid[grid == self.EXTENDING] = self.WALL
                    return

            if not (directions := [d for d in self.extendable_directions(grid, x, y)]):
                grid[grid == self.EXTENDING] = self.PASSAGE
                x, y = org_x, org_y
                continue

            d = random.choice(directions)
            grid[y + d[1], x + d[0]] = self.EXTENDING
            x += d[0] * 2
            y += d[1] * 2