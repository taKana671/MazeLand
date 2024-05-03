import pathlib

from panda3d.bullet import BulletRigidBodyNode, BulletPlaneShape
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletSphereShape, BulletHeightfieldShape, ZUp
from panda3d.core import NodePath, PandaNode
from panda3d.core import Vec3, Point3, Vec2, CardMaker
from panda3d.core import Filename, PNMImage
from panda3d.core import Shader
from panda3d.core import TextureStage, TransformState
from panda3d.core import GeoMipTerrain

from panda3d.core import BitMask32, Vec3, LColor



class Sky(NodePath):

    def __init__(self):
        super().__init__(PandaNode('sky'))
        model = base.loader.load_model('models/blue-sky/blue-sky-sphere')
        model.set_color(LColor(2, 2, 2, 1))
        model.set_scale(0.2)
        model.set_z(0)
        model.reparent_to(self)
        self.set_shader_off()


class Terrain(NodePath):

    def __init__(self):
        super().__init__(BulletRigidBodyNode('terrain'))
        self.file_path = 'terrains/heightfield.png'
        self.heigt = 30

        self.set_pos(Point3(0, 0, 0))
        self.node().set_mass(0)
        self.set_collide_mask(BitMask32.bit(1))

        textures = [
            'textures/grass.jpg',
            'textures/grass_01.png',
            'textures/grass_02.jpg',
            'textures/grass_03.jpg',
        ]
        self.add_shape_to_terrain()
        self.make_geomip_terrain()
        self.setup_shader()
        self.setup_textures(textures)

    def add_shape_to_terrain(self):
        img = PNMImage(Filename(self.file_path))
        shape = BulletHeightfieldShape(img, self.heigt, ZUp)
        shape.set_use_diamond_subdivision(True)
        self.node().add_shape(shape)

    def make_geomip_terrain(self):
        self.terrain = GeoMipTerrain('geomip_terrain')
        self.terrain.set_heightfield(self.file_path)
        self.terrain.set_border_stitching(True)

        self.terrain.set_block_size(8)
        self.terrain.set_min_level(2)
        self.terrain.set_focal_point(base.camera)

        pos = Point3(-128, -128, -(self.heigt / 2))
        self.root = self.terrain.get_root()
        self.root.set_scale(Vec3(1, 1, self.heigt))
        self.root.set_pos(pos)
        self.terrain.generate()
        self.root.reparent_to(self)

    def setup_shader(self):
        shader = Shader.load(
            Shader.SL_GLSL,
            'shaders/terrain_v.glsl',
            'shaders/terrain_f.glsl'
        )
        self.root.set_shader(shader)

    def setup_textures(self, textures):
        self.root.clear_texture()

        for i, img_file in enumerate(textures):
            ts = TextureStage(f'ts{i}')
            ts.set_sort(i)
            self.root.set_shader_input(f'tex_ScaleFactor{i}', 10)  # 10 is texture scale
            tex = base.loader.load_texture(img_file)
            self.root.set_texture(ts, tex)


class Scene:

    def __init__(self, world):
        self.world = world

        self.scene = NodePath('scene')
        self.scene.reparent_to(base.render)

        self.sky = Sky()
        self.sky.reparent_to(self.scene)

        self.terrain = Terrain()
        self.terrain.reparent_to(self.scene)
        self.world.attach(self.terrain.node())