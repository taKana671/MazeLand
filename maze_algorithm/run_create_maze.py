from cymaze.wall_extending import WallExtendingAlgorithm
# from pymaze.wall_extending import WallExtendingAlgorithm


def main():
    grid = WallExtendingAlgorithm(155, 155).create_maze()
    print(grid)
    # return grid


if __name__ == '__main__':
    main()

# cymaze
# In [1]: from run_create_maze import main

# In [2]: %timeit main()
# 5.32 ms ± 189 μs per loop (mean ± std. dev. of 7 runs, 100 loops each)

# pymaze
# In [1]: from run_create_maze import main

# In [2]: %timeit main()
# 39.3 ms ± 3.08 ms per loop (mean ± std. dev. of 7 runs, 10 loops each)