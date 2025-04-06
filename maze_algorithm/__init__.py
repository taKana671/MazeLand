try:
    from maze_algorithm.cymaze.wall_extending import WallExtendingAlgorithm
    print('Use cython code.')
except ImportError:
    from maze_algorithm.pymaze.wall_extending import WallExtendingAlgorithm
    print('Use python code.')