from typing import NamedTuple
from enum import Enum, auto

from panda3d.bullet import BulletRigidBodyNode, BulletBoxShape
from panda3d.core import NodePath, TextureStage
from panda3d.core import Vec3, Point3, BitMask32, Point2

from create_maze_2d import WallExtendingAlgorithm
from create_geomnode import Cube


class Corners(Enum):

    TOP_LEFT = auto()
    BOTTOM_LEFT = auto()
    TOP_RIGHT = auto()
    BOTTOM_RIGHT = auto()


class Block(NodePath):

    def __init__(self, name, pos, scale, mask):
        super().__init__(BulletRigidBodyNode(name))
        cube = Cube()
        self.model = self.attach_new_node(cube.node())

        self.set_pos(pos)
        self.set_scale(scale)
        self.set_collide_mask(mask)

        end, tip = self.model.get_tight_bounds()
        self.node().add_shape(BulletBoxShape((tip - end) / 2))
        self.node().set_mass(0)


class Space(NamedTuple):

    row: int
    col: int


class MazeBuilder:

    def __init__(self, world, parent):
        self.world = world
        self.wall_size = Vec3(2, 2, 4)
        self.np_walls = NodePath('walls')
        self.np_walls.reparent_to(parent)
        self.np_walls.set_pos(0, 0, -12)

    def get_maze_pos(self):
        return self.np_walls.get_pos()

    def get_entrance(self):
        return self.space_to_cartesian(*self.entrance)

    def get_exit(self):
        return self.space_to_cartesian(*self.exit)

    def space_to_cartesian(self, row, col):
        x = (col - self.cols // 2) * self.wall_size.x
        y = (-row + self.rows // 2) * self.wall_size.y
        return Point2(x, y)

    def setup(self, rows, cols):
        """Build a maze.
            Args:
                rows (int): the number of rows; must be odd.
                cols (int): the number of columns; must be odd.
        """
        self.rows = rows if rows % 2 != 0 else rows - 1
        self.cols = cols if cols % 2 != 0 else cols - 1

        self.top_left = self.space_to_cartesian(0, 0)
        self.bottom_right = self.space_to_cartesian(self.rows - 1, self.cols - 1)
        self.top_right = self.space_to_cartesian(0, self.cols - 1)
        self.bottom_left = self.space_to_cartesian(self.rows - 1, 0)
        self.entrance = (0, 1)
        self.exit = (self.rows - 1, self.cols - 2)

        self.build()

    def build(self):
        tex_brick = base.loader.load_texture('textures/brick.jpg')
        tex_stone = base.loader.load_texture('textures/concrete2.jpg')
        # config = types.SimpleNamespace(wall=1, aisle=0, extending=2)

        np_brick = NodePath('brick')
        np_stone = NodePath('stone')
        np_closure = NodePath('closure')

        for np in [np_brick, np_stone, np_closure]:
            np.reparent_to(self.np_walls)

        grid = WallExtendingAlgorithm(self.rows, self.cols).create_maze()
        stone_size = Vec3(self.wall_size.xy, 0.25)
        brick_z = self.wall_size.z / 2
        stone_z = self.wall_size.z + stone_size.z / 2

        for r in range(self.rows):
            for c in range(self.cols):
                if grid[r, c] == WallExtendingAlgorithm.WALL:
                    xy = self.space_to_cartesian(r, c)

                    match (r, c):
                        case self.exit:
                            mask = BitMask32.bit(3)
                            hide = True
                        case self.entrance:
                            mask = BitMask32.bit(2) | BitMask32.bit(4)
                            hide = True
                        case _:
                            mask = BitMask32.bit(2) | BitMask32.bit(4)
                            hide = False

                    self.make_block(f'brick_{r}_{c}', Point3(xy, brick_z), self.wall_size, mask, hide, np_brick)
                    self.make_block(f'top_{r}_{c}', Point3(xy, stone_z), stone_size, mask, hide, np_stone)

        su = (self.wall_size.x * 2 + self.wall_size.y * 2) / 4
        sv = self.wall_size.z / 4
        np_brick.set_tex_scale(TextureStage.get_default(), su, sv)

        np_brick.set_texture(tex_brick)
        np_stone.set_texture(tex_stone)

    def make_block(self, name, pos, scale, mask, hide=False, parent=None):
        if parent is None:
            parent = self.np_walls.find('closure')

        block = Block(name, pos, scale, mask)
        block.reparent_to(parent)
        self.world.attach(block.node())

        if hide:
            block.hide()

        return block

    def is_outside(self, pt2):
        upper = self.top_left.y + self.wall_size.y / 2
        lower = self.bottom_right.y - self.wall_size.y / 2

        if pt2.y < lower or pt2.y > upper:
            return True

    def destroy(self):
        for np in self.np_walls.get_children():
            for block in np.get_children():
                self.world.remove(block.node())
                block.remove_node()
            np.remove_node()
