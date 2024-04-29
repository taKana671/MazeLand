import sys

from panda3d.bullet import BulletWorld, BulletSphereShape
from panda3d.bullet import BulletDebugNode, BulletRigidBodyNode
from panda3d.core import Vec3, NodePath, BitMask32, Point3
from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
from panda3d.core import load_prc_file_data

from create_maze_3d import MazeBuilder
from lights import BasicAmbientLight, BasicDayLight
from scene import Scene
from aircraft import AircraftController


load_prc_file_data("", """
    textures-power-2 none
    gl-coordinate-system default
    window-title Panda3D Avoid Balls
    filled-wireframe-apply-shader true
    stm-max-views 8
    stm-max-chunk-count 2048""")


class Maze3D(ShowBase):

    def __init__(self):
        super().__init__()
        self.disable_mouse()
        self.world = BulletWorld()
        self.world.set_gravity(Vec3(0, 0, -9.81))

        self.debug = self.render.attach_new_node(BulletDebugNode('debug'))
        self.world.set_debug_node(self.debug.node())

        self.maze_builder = MazeBuilder(self.world, 21, 21)
        self.maze_builder.build()

        self.aircraft_controller = AircraftController(self.world, self.maze_builder)

        self.floater = NodePath('floater')
        self.floater.set_z(5)
        self.floater.reparent_to(self.aircraft_controller.aircraft.root_np)

        self.camera.reparent_to(self.aircraft_controller.aircraft.root_np)
        self.camera.set_pos(self.aircraft_controller.get_relative_pos(Point3(0, -2, 10)))  # Point3(0, -8, 6)
        self.camera.look_at(self.floater)
        self.camLens.set_fov(90)

        # self.camera.set_pos(0, 0, 1000)
        # self.camera.look_at(0, 0, 0)

        self.ambient_light = BasicAmbientLight()
        self.ambient_light.reparent_to(self.render)
        self.day_light = BasicDayLight()
        self.day_light.reparent_to(self.render)
        self.scene = Scene(self.world)

        self.accept('escape', sys.exit)
        self.accept('d', self.toggle_debug)
        self.taskMgr.add(self.update, 'update')

    def toggle_debug(self):
        if self.debug.is_hidden():
            self.debug.show()
        else:
            self.debug.hide()

    def update(self, task):
        dt = globalClock.get_dt()
        self.aircraft_controller.update(dt)
        self.world.do_physics(dt)
        return task.cont


if __name__ == '__main__':
    app = Maze3D()
    app.run()
