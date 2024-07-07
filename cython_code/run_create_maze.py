from create_maze2d_cy import WallExtendingAlgorithm


def main():
    grid = WallExtendingAlgorithm(155, 155).create_maze()
    print(grid)
    return grid


if __name__ == '__main__':
    main()


# python code
# In [2]: %timeit main()
# 36.4 ms ± 1.14 ms per loop (mean ± std. dev. of 7 runs, 10 loops each)


# Changed extension from py to pyx,  and stopped use walrus operator and match.
# In [6]: %timeit main()
# 26.9 ms ± 1.27 ms per loop (mean ± std. dev. of 7 runs, 10 loops each)

# Changed def extendable_directions and extend_wall to cdef methods。
# In [5]: %timeit main()
# 23.6 ms ± 956 µs per loop (mean ± std. dev. of 7 runs, 10 loops each)

# Changed list to C array.
# In [3]: %timeit main()
# 22.2 ms ± 330 µs per loop (mean ± std. dev. of 7 runs, 10 loops each)

# type ndarray
# In [4]: %timeit main()
# 21.5 ms ± 379 µs per loop (mean ± std. dev. of 7 runs, 10 loops each)

# Changed to use memoryviewとndarray in extend_wall.
#  In [4]: %timeit main()
# 20.4 ms ± 319 µs per loop (mean ± std. dev. of 7 runs, 100 loops each)

# Changed to use random.choice, depending on the length of list.
# In [6]: %timeit main()
# 18.2 ms ± 144 µs per loop (mean ± std. dev. of 7 runs, 100 loops each)