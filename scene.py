import pathlib

from panda3d.bullet import BulletRigidBodyNode, BulletPlaneShape, BulletConvexHullShape
from panda3d.bullet import BulletRigidBodyNode, BulletSoftBodyNode
from panda3d.bullet import BulletSphereShape, BulletHeightfieldShape, ZUp
from panda3d.bullet import BulletHelper
from panda3d.core import NodePath, PandaNode
from panda3d.core import Vec3, Point3, Vec2, CardMaker
from panda3d.core import Filename, PNMImage
from panda3d.core import Shader
from panda3d.core import TextureStage, TransformState
from panda3d.core import GeoMipTerrain
from panda3d.core import GeomNode, GeomVertexFormat
from panda3d.core import BitMask32, Vec3, LColor

from create_geomnode import Cylinder
from create_maze_3d import MazeBuilder, Space
from lights import BasicAmbientLight, BasicDayLight


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


class Cloth(NodePath):

    def __init__(self, info, p00, p10, p01, p11, resx, resy, fixeds, gendiags):
        super().__init__(BulletSoftBodyNode.make_patch(info, p00, p10, p01, p11, resx, resy, fixeds, gendiags))
        material = self.node().append_material()
        material.set_linear_stiffness(0.4)
        self.node().generate_bending_constraints(2, material)
        self.node().set_total_mass(50.0)
        self.node().get_shape(0).set_margin(0.5)
        self.set_name('banner')
        self.set_collide_mask(BitMask32.all_on())

        fmt = GeomVertexFormat.getV3n3t2()
        geom = BulletHelper.make_geom_from_faces(self.node(), fmt, True)
        self.node().link_geom(geom)

        vis_nd = GeomNode('visualization')
        vis_nd.add_geom(geom)
        vis_np = self.attach_new_node(vis_nd)
        vis_np.reparent_to(self)
        vis_np.set_texture(base.loader.load_texture('textures/finish.png'))
        BulletHelper.make_texcoords_for_patch(geom, resx, resy)


class Poles(NodePath):

    def __init__(self, gate_w, pole_h):
        super().__init__(BulletRigidBodyNode('poles'))
        self.set_collide_mask(BitMask32.bit(1))
        half = gate_w / 2
        self.left = self.assemble(Point3(-half, 0, 0), pole_h)
        self.right = self.assemble(Point3(half, 0, 0), pole_h)

    def assemble(self, pos, pole_h):
        cylinder = Cylinder(radius=0.15, height=pole_h, segs_a=12)
        cylinder.set_pos(pos)
        cylinder.reparent_to(self)
        shape = BulletConvexHullShape()
        shape.add_geom(cylinder.node().get_geom(0))
        self.node().add_shape(shape, TransformState.make_pos(pos))

        return cylinder


class GoalGate(NodePath):

    # def __init__(self, world, gate_pos, gate_angle=180, gate_w=4, pole_h=4):
    def __init__(self, world, gate_w=4, pole_h=4):
        super().__init__(PandaNode('goal_gate'))
        self.world = world
        self.gate_w = gate_w
        self.pole_h = pole_h
        self.create_poles()
        # self.set_h(gate_angle)
        # self.set_pos(gate_pos)

        # self.create_poles(gate_w, pole_h)

    def setup(self, gate_pos, gate_angle=180):
        self.set_h(gate_angle)
        # self.set_pos(gate_pos)
        self.poles.set_pos(base.render, gate_pos)
        self.create_banner()

    def destroy(self):
        self.world.remove(self.banner.node())
        self.banner.remove_node()

    def create_poles(self):
        self.poles = Poles(self.gate_w, self.pole_h)
        tex = base.loader.load_texture('textures/concrete2.jpg')
        self.poles.set_texture(tex)
        self.poles.reparent_to(self)
        self.world.attach(self.poles.node())

    # def create_banner(self, pole_h):
    def create_banner(self):
        info = self.world.get_world_info()
        info.set_air_density(1.2)
        info.set_water_density(0)
        info.set_water_offset(0)
        info.set_water_normal(Vec3(0, 0, 0))

        resx = 4
        resy = 4

        left_w_pt = self.poles.left.get_pos(base.render)
        right_w_pt = self.poles.right.get_pos(base.render)

        p00 = left_w_pt + Vec3(0, 0, self.pole_h - 1)   # bottom left
        p01 = right_w_pt + Vec3(0, 0, self.pole_h - 1)  # top left
        p10 = left_w_pt + Vec3(0, 0, self.pole_h)       # bottom right
        p11 = right_w_pt + Vec3(0, 0, self.pole_h)      # top right

        fixeds = 1 + 2 + 4 + 8
        gendiags = True

        self.banner = Cloth(info, p00, p10, p01, p11, resx, resy, fixeds, gendiags)
        self.world.attach(self.banner.node())
        self.banner.reparent_to(self)


class Scene:

    def __init__(self, world):
        self.world = world

        self.ambient_light = BasicAmbientLight()
        self.day_light = BasicDayLight()

        self.scene = NodePath('scene')
        self.scene.reparent_to(base.render)

        self.sky = Sky()
        self.sky.reparent_to(self.scene)

        self.terrain = Terrain()
        self.terrain.reparent_to(self.scene)
        self.world.attach(self.terrain.node())

        self.maze = MazeBuilder(self.world, self.scene)
        gate_w = self.maze.wall_size.x * 2
        self.goal_gate = GoalGate(self.world, gate_w=gate_w)
        self.goal_gate.reparent_to(self.scene)

    def build_maze(self, rows=21, cols=21):
        self.maze.setup(rows, cols)
        # make goal gate.
        xy = self.maze.get_exit()
        gate_pos = Point3(xy, self.maze.get_maze_pos().z + 2)
        self.goal_gate.setup(gate_pos)

    def destroy_maze(self):
        self.maze.destroy()
        self.goal_gate.destroy()
