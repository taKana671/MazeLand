# cython: profile=True
import random
cimport numpy as cnp
import numpy as np

from cython cimport boundscheck, wraparound
from cython.parallel cimport prange


DEF WALL = 1
DEF PASSAGE = 0
DEF EXTENDING = 2


cdef class WallExtendingAlgorithm:

    cdef:
        int rows, cols 


    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols

    def create_maze(self):
        grid = np.zeros((self.rows, self.cols), dtype=np.intc)
        grid[[0, -1]] = WALL
        grid[:, [0, -1]] = WALL

        starts = [pt for pt in self.starts_pts()]
        random.shuffle(starts)

        for x, y in starts:
            if grid[y, x] != WALL:
                self.extend_wall(grid, x, y)

        # print(grid)
        return grid

    def starts_pts(self):
        for y in range(1, self.rows - 1):
            for x in range(1, self.cols - 1):
                if y % 2 == 0 and x % 2 == 0:
                    yield (x, y)

    @boundscheck(False)
    @boundscheck(False)
    cdef list extendable_directions(self, int[:, ::1] grid, int[4][2] directions, int x, int y):
        cdef:
            int i, ahead_2x, ahead_2y
            int[2] direction
            list extendables = []

        for i in range(4):
            direction = directions[i]
            ahead_2x = x + direction[0] * 2
            ahead_2x = x + direction[1] * 2
        
            if grid[ahead_2y, ahead_2x] != EXTENDING:
                extendables.append(i)
        
        return extendables

    @boundscheck(False)
    @boundscheck(False)
    def extend_wall(self, cnp.ndarray grid, int org_x, int org_y):

        cdef:
            int x = org_x
            int y = org_y
            int[:, ::1] grid_memv = grid
            int[4][2] directions = [[0, 1], [0, -1], [1, 0], [-1, 0]]
            int[2] d
            int i

        while True:

            if grid_memv[y, x] == PASSAGE:
                grid_memv[y, x] = EXTENDING
            elif grid_memv[y, x] == WALL:
                grid[grid == EXTENDING] = WALL
                return

            extendables = self.extendable_directions(grid, directions, x, y)

            if not extendables:
                grid[grid == EXTENDING] = PASSAGE
                x, y = org_x, org_y
                continue

            i = random.choice(extendables)
            d = directions[i]

            grid_memv[y + d[1], x + d[0]] = EXTENDING
            x += d[0] * 2
            y += d[1] * 2