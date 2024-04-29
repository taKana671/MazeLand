from typing import NamedTuple
from panda3d.bullet import BulletRigidBodyNode, BulletBoxShape
from panda3d.core import NodePath, TextureStage
from panda3d.core import Vec3, Point3, BitMask32, Point2

from create_maze_2d import WallExtendingAlgorithm
from create_geomnode import Cube


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

    def __init__(self, world, rows, cols, side=2):
        self.world = world
        self.wall_wd = Point2(side)
        self.rows = rows if rows % 2 != 0 else rows - 1
        self.cols = cols if cols % 2 != 0 else cols - 1

        self.entrance = Space(self.rows - 1, self.cols - 2)
        self.exit = Space(0, 1)

        self.np_walls = NodePath('walls')
        self.np_walls.reparent_to(base.render)
        # self.walls.set_pos(0, 0, -3)  # terrainの高さに合わせて配置するときは、droneの高さも設定しなおす

    def get_entrance(self):
        return self.space_to_cartesian(*self.entrance)

    def space_to_cartesian(self, row, col):
        x = (col - self.cols // 2) * self.wall_wd.x
        y = (-row + self.rows // 2) * self.wall_wd.y
        return Point2(x, y)

    def build(self):
        tex_brick = base.loader.load_texture('textures/brick.jpg')
        tex_stone = base.loader.load_texture('textures/concrete2.jpg')
        # config = types.SimpleNamespace(wall=1, aisle=0, extending=2)

        np_brick = NodePath('brick')
        np_brick.reparent_to(self.np_walls)
        np_stone = NodePath('stone')
        np_stone.reparent_to(self.np_walls)

        grid = WallExtendingAlgorithm(self.rows, self.cols).create_maze()
        brick_size = Vec3(self.wall_wd, self.wall_wd.x * 2)
        stone_size = Vec3(self.wall_wd, 0.5)
        brick_z = brick_size.z / 2
        stone_z = brick_size.z + stone_size.z / 2

        for r in range(self.rows):
            for c in range(self.cols):
                if grid[r, c] == WallExtendingAlgorithm.WALL:
                    xy = self.space_to_cartesian(r, c)

                    match (r, c):
                        case self.entrance:
                            mask = BitMask32.bit(1) | BitMask32.bit(2)
                            hide = True
                        case self.exit:
                            mask = BitMask32.bit(1) | BitMask32.bit(3)
                            hide = True
                        case _:
                            mask = BitMask32.bit(1) | BitMask32.bit(2)
                            hide = False

                    self.make_block(f'brick_{r}_{c}', Point3(xy, brick_z), brick_size, mask, hide, np_brick)
                    self.make_block(f'top_{r}_{c}', Point3(xy, stone_z), stone_size, mask, hide, np_stone)

        su = (brick_size.x * 2 + brick_size.y * 2) / 4
        sv = brick_size.z / 4
        np_brick.set_tex_scale(TextureStage.get_default(), su, sv)

        np_brick.set_texture(tex_brick)
        np_stone.set_texture(tex_stone)
        self.np_walls.flatten_strong()

    def make_block(self, name, pos, scale, mask, hide=False, parent=None):
        if parent is None:
            parent = self.np_walls

        block = Block(name, pos, scale, mask)
        block.reparent_to(parent)
        self.world.attach(block.node())

        if hide:
            block.hide()
