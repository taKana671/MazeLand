import sys
from collections import deque
from datetime import datetime
from enum import IntEnum, Enum, auto

from panda3d.bullet import BulletWorld, BulletSphereShape
from panda3d.bullet import BulletDebugNode, BulletRigidBodyNode
from panda3d.core import Vec3, NodePath, BitMask32, Point3, Quat, LColor, Vec2
from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
from direct.showbase.InputStateGlobal import inputState
from panda3d.core import load_prc_file_data
from panda3d.core import Camera

import numpy as np

from create_maze_3d import MazeBuilder, Space
from lights import BasicAmbientLight, BasicDayLight
from scene import Scene
from aircraft import AircraftController
from maze_walker import MazeWalkerController
from basic_character import Direction
from create_geomnode import SphericalShape


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

        self.scene = Scene(self.world)

        self.maze_builder = MazeBuilder(self.world, 21, 21)
        self.maze_builder.build()

        self.walker_controller = MazeWalkerController(self.world, self.maze_builder)
        # self.floater = NodePath('floater')
        # self.floater.set_z(1)   # 3
        # self.floater.reparent_to(self.walker_controller.walker.root_np)

        # ################not use camera controller#############################
        # self.camera.reparent_to(self.walker_controller.walker.direction_np)
        # self.camera.set_pos(self.walker_controller.navigate(Point3(0, 1, 2)))  # Point3(0, -8, 6)
        # self.camera.look_at(self.floater)
        # self.camLens.set_fov(90)
        # ######################################################################

        # ############### aircraft part ############################
        self.aircraft_controller = AircraftController(self.world, self.maze_builder)

        # self.floater = NodePath('floater')
        # self.floater.set_z(5)
        # self.floater.reparent_to(self.aircraft_controller.aircraft.root_np)

        # self.camera.reparent_to(self.aircraft_controller.aircraft.root_np)
        # self.camera.set_pos(self.aircraft_controller.get_relative_pos(Point3(0, -2, 10)))  # Point3(0, -8, 6)
        # self.camera.look_at(self.floater)
        # self.camLens.set_fov(90)

        # self.camera.set_pos(0, 0, 1000)
        # self.camera.look_at(0, 0, 0)
        # ###########################################

        self.ambient_light = BasicAmbientLight()
        self.ambient_light.reparent_to(self.render)
        self.day_light = BasicDayLight()
        self.day_light.reparent_to(self.render)


        self.display_regions2()
        # self.split_screen()

        inputState.watch_with_modifiers('forward', 'arrow_up')
        inputState.watch_with_modifiers('backward', 'arrow_down')
        inputState.watch_with_modifiers('left', 'arrow_left')
        inputState.watch_with_modifiers('right', 'arrow_right')
        inputState.watch_with_modifiers('jump', 'enter')

        self.accept('escape', sys.exit)
        self.accept('d', self.toggle_debug)
        self.accept('p', self.print_info)
        self.taskMgr.add(self.update, 'update')

    def split_screen(self):
        pos = self.aircraft_controller.get_relative_pos(Point3(0, -2, 10))
        cam1 = self.create_cameras((0.0, 0.5, 0.0, 1), pos)
        floater = NodePath('floater_1')
        floater.set_z(5)
        floater.reparent_to(self.aircraft_controller.aircraft.root_np)
        cam1.reparent_to(self.aircraft_controller.aircraft.root_np)
        cam1.look_at(floater)

        pos = self.walker_controller.navigate(Point3(0, 1, 2))
        cam2 = self.create_cameras((0.5, 1.0, 0, 1), pos)
        floater = NodePath('floater_2')
        floater.set_z(1)   # 3
        floater.reparent_to(self.walker_controller.walker.root_np)
        cam2.reparent_to(self.walker_controller.walker.direction_np)
        cam2.look_at(floater)

        self.camNode.set_active(False)

    def create_cameras(self, region, pos):
        camera = self.make_camera(self.win, displayRegion=region)
        camera.node().get_lens().set_aspect_ratio(3.0 / 4.0)
        camera.node().get_lens().set_fov(90)
        camera.set_pos(pos)
        return camera

    def display_regions2(self):
        self.w_floater = NodePath('floater')
        self.w_floater.set_z(1)   # 3
        self.w_floater.reparent_to(self.walker_controller.walker.root_np)

        ################not use camera controller#############################
        self.camera.reparent_to(self.walker_controller.walker.direction_np)
        self.camera.set_pos(self.walker_controller.navigate(Point3(0, 1, 2)))  # Point3(0, -8, 6)
        # self.camera.look_at(self.walker_controller.walker.character)
        self.camera.look_at(self.w_floater)
        self.camLens.set_fov(90)

        # region = self.win.make_display_region((0., 0.25, 0.75, 1)) # 左上
        region = self.win.make_display_region((0.75, 1, 0.75, 1)) # 右上
        region.set_sort(100)
        cam_np = NodePath(Camera('cam'))
        cam_np.node().get_lens().set_aspect_ratio(3.0 / 4.0)
        cam_np.node().get_lens().set_fov(90)
        region.set_camera(cam_np)

        self.floater = NodePath('floater')
        self.floater.set_z(5)
        self.floater.reparent_to(self.aircraft_controller.aircraft.root_np)
        cam_np.reparent_to(self.aircraft_controller.aircraft.root_np)
        cam_np.set_pos(self.aircraft_controller.get_relative_pos(Point3(0, -2, 6)))  # Point3(0, -8, 6)
        cam_np.look_at(self.floater)


    def toggle_debug(self):
        if self.debug.is_hidden():
            self.debug.show()
        else:
            self.debug.hide()

    def print_info(self):
        walker_pos = self.walker_controller.get_walker_pos()
        print('walker_pos', walker_pos)
        print('camera_pos', self.camera.get_pos(self.render))
        print('backward_pos', self.walker_controller.navigate(Vec3(0, 2, 3)) + walker_pos)
        print('camera reative pos', self.camera.get_pos())
        print('relative_pos', self.walker_controller.navigate(Vec3(0, -1, 0)))
        print('forwad_vector', self.walker_controller.walker.direction_np.get_quat(base.render).get_forward())
        print('camera_forwad_vector', self.camera.get_quat(base.render).get_forward())

    def control_walker(self, dt):
        direction = None

        if inputState.is_set('forward'):
            direction = Direction.FORWARD
        elif inputState.is_set('backward'):
            direction = Direction.BACKWARD
        elif inputState.is_set('left'):
            direction = Direction.LEFTWARD
        elif inputState.is_set('right'):
            direction = Direction.RIGHTWARD
        elif inputState.is_set('jump'):
            direction = Direction.UPWARD

        camera_pos = self.walker_controller.update(direction, dt)
        return camera_pos

    def update(self, task):
        dt = globalClock.get_dt()
        self.control_walker(dt)

        self.aircraft_controller.update(dt)
        self.world.do_physics(dt)
        return task.cont


if __name__ == '__main__':
    app = Maze3D()
    app.run()
