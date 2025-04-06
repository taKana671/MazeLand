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
            int[:, ::1] grid, starts
            int[::1] random_idx
            list li
            int x, y, i, r, c, length
        
        grid = np.zeros((self.rows, self.cols), dtype=np.intc)
        
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
        li = list(range(length))
        random.shuffle(li)
        random_idx = np.array(li, dtype=np.intc)
        self.starts_pts(starts, random_idx)

        for i in range(length):
            x = starts[i][0]
            y = starts[i][1]

            if grid[y, x] != WALL:
                self.extend_wall(grid, &x, &y)

        #print(grid)
        return np.array(grid)

    @wraparound(False)
    @boundscheck(False)
    cdef starts_pts(self, int[:, ::1] starts, int[::1] random_idx):
        cdef:
            int y, x, pos
            int i = 0
        
        for y in range(1, self.rows - 1):
            for x in range(1, self.cols - 1):
                if fmod(y, 2) == 0.0 and fmod(x, 2) == 0:
                    pos = random_idx[i]
                    starts[pos][0] = x
                    starts[pos][1] = y
                    i += 1

    @wraparound(False)
    @boundscheck(False)
    cdef int extendable_directions(
            self, int[:, ::1] grid, int[4][2] *directions, int[4] *extendables, int *x, int *y):
        cdef:
            int i, ahead_2x, ahead_2y
            int cnt = 0

        for i in range(4):
            ahead_2x = x[0] + directions[0][i][0] * 2
            ahead_2y = y[0] + directions[0][i][1] * 2
        
            if grid[ahead_2y, ahead_2x] != EXTENDING:
                extendables[0][cnt] = i
                cnt += 1
        
        return cnt

    @boundscheck(False)
    @wraparound(False)
    cdef extend_wall(self, int[:, ::1] grid, int *org_x, int *org_y):

        cdef:
            int x = org_x[0]
            int y = org_y[0]

            int min_x = org_x[0]
            int max_x = org_x[0]
            int min_y = org_y[0]
            int max_y = org_y[0]

            int[4][2] directions = [[0, 1], [0, -1], [1, 0], [-1, 0]]
            int[4] extendables
            int idx, j, i, n, cnt, dx, dy

        while True:

            if grid[y, x] == PASSAGE:
                grid[y, x] = EXTENDING

            elif grid[y, x] == WALL:
                #for j in range(self.rows):
                #    for i in range(self.cols):
                for j in range(min_y, max_y + 1):
                    for i in range(min_x, max_x + 1):
                        if grid[j, i] == EXTENDING:
                            grid[j, i] = WALL
                return

            cnt = self.extendable_directions(grid, &directions, &extendables, &x, &y)

            if cnt == 0:
                #for j in range(self.rows):
                #    for i in range(self.cols):
                for j in range(min_y, max_y + 1):
                    for i in range(min_x, max_x + 1):
                        if grid[j, i] == EXTENDING:
                            grid[j, i] = PASSAGE

                x, y = org_x[0], org_y[0]
                continue

            if cnt == 1:
                idx = extendables[cnt - 1]
            else:
                n = random.choice(range(cnt))
                idx = extendables[n]
            
            dx = directions[idx][0]
            dy = directions[idx][1]

            grid[y + dy, x + dx] = EXTENDING
            x += dx * 2
            y += dy * 2

            if max_x < x:
                max_x = x

            if min_x > x:
                min_x = x

            if max_y < y:
                max_y = y

            if min_y > y:
                min_y = y
            