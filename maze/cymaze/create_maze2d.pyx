# cython: language_level=3


import random

import numpy as np
cimport numpy as np
from cython cimport boundscheck, wraparound
from cython.view cimport array as cvarray
from libc.math cimport fmod


DEF WALL = 1
DEF PASSAGE = 0
DEF EXTENDING = 2


cdef class WallExtendingAlgorithm:

    cdef:
        int rows, cols

    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols

    @wraparound(False)
    @boundscheck(False)
    cpdef np.ndarray create_maze(self):
        cdef:
            # int[:, ::1] grid = np.zeros((self.rows, self.cols), dtype=np.intc)
            int[:, ::1] grid, starts
            int[:, :1] random_idx
            list li
            int x, y, i, r, c, length
        
        grid = cvarray(shape=(self.rows, self.cols), itemsize=sizeof(int), format='i')
        
        r = self.rows - 1
        for i in range(self.cols):
            grid[0, i] = 1
            grid[r, i] = 1

        c = self.cols - 1
        for i in range(self.rows):
            grid[i, 0] = 1
            grid[i, c] = 1

        r = (self.rows - 2) // 2
        c = (self.cols - 2) // 2
        length = r * c

        starts = cvarray(shape=(length, 2), itemsize=sizeof(int), format='i')
        li = random.shuffle(range(length))
        random_idx = np.array(li, dtype=np.intc)
        self.starts_pts(random_idx, idxes)

        for i in range(length):
            x = starts[i][0]
            y = starts[i][1]

            if grid[y, x] != WALL:
                self.extend_wall(grid, x, y)

        #print(grid)
        return np.array(grid)

    cdef starts_pts(self, int[:, ::1] starts, int[:, :1] random_idx):
        cdef:
            int y, x, i, pos
        
        i = 0
        for y in range(1, self.rows - 1):
            for x in range(1, self.cols - 1):
                if fmod(y, 2) == 0.0 and fmod(x, 2) == 0:
                    pos = random_idx[i]
                    starts[pos][0] = x
                    starts[pos][1] = y
                    i += 1

    @wraparound(False)
    @boundscheck(False)
    cdef list extendable_directions(self, int[:, ::1] grid, int[4][2] *directions, int x, int y):
        cdef:
            int i, ahead_2x, ahead_2y
            # int[2] direction
            list extendables = []

        for i in range(4):
            # direction = directions[0][i]
            ahead_2x = x + directions[0][i][0] * 2
            ahead_2y = y + directions[0][i][1] * 2
        
            if grid[ahead_2y, ahead_2x] != EXTENDING:
                extendables.append(i)
        
        return extendables

    @boundscheck(False)
    @wraparound(False)
    #cdef extend_wall(self, np.ndarray grid, int org_x, int org_y):
    cdef extend_wall(self, int[:, ::1] grid, int org_x, int org_y):

        cdef:
            int x = org_x
            int y = org_y
            # int[:, ::1] grid_memv = grid
            int[4][2] directions = [[0, 1], [0, -1], [1, 0], [-1, 0]]
            int[2] d
            int idx, j, i
            list extendables

        while True:

            if grid[y, x] == PASSAGE:
                grid[y, x] = EXTENDING

            elif grid[y, x] == WALL:

                for j in range(self.rows):
                    for i in range(self.cols):
                        if grid[j, i] == EXTENDING:
                            grid[j, i] = WALL
                return

            extendables = self.extendable_directions(grid, &directions, x, y)

            if not extendables:
                for j in range(self.rows):
                    for i in range(self.cols):
                        if grid[j, i] == EXTENDING:
                            grid[j, i] = PASSAGE

                x, y = org_x, org_y
                continue

            if len(extendables) == 1:
                idx = extendables[0]
            else:
                idx = random.choice(extendables)
            
            d = directions[idx]

            grid[y + d[1], x + d[0]] = EXTENDING
            x += d[0] * 2
            y += d[1] * 2