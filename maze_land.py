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
from panda3d.core import load_prc_file_data, TransformState
from panda3d.core import Camera, OrthographicLens

from create_maze_3d import MazeBuilder, Space
from lights import BasicAmbientLight, BasicDayLight
from scene import Scene
from aircraft import AircraftController
from maze_walker import MazeWalkerController
from basic_character import Direction, Status


load_prc_file_data("", """
    textures-power-2 none
    gl-coordinate-system default
    window-title Panda3D Avoid Balls
    filled-wireframe-apply-shader true
    stm-max-views 8
    stm-max-chunk-count 2048""")


class CameraController:

    def __init__(self, camera, walker_controller):
        self.camera = camera
        self.walker_controller = walker_controller

        self.floater = NodePath('floater')
        self.floater.set_z(0.5)   # 3
        self.floater.reparent_to(self.walker_controller.walker.root_np)

        self.state = Status.STOP
        self.total_distance = 0
        self.trace_q = deque()

    @property
    def camera_z(self):
        return self.walker_controller.get_walker_pos().z + 1

    def setup_camera(self):
        walker_pos = self.walker_controller.get_walker_pos()
        pos = self.walker_controller.navigate(Point3(0, 2.0, 1)) + walker_pos
        self.trace_q.append(pos.xy)
        self.camera.set_pos(pos)
        self.camera.look_at(self.floater)

    def change_z(self):
        self.camera.set_z(self.camera_z)

    def move(self, dt, max_distance=2):
        distance = dt * 2
        camera_xy = self.camera.get_pos().xy

        if (total := self.total_distance + distance) >= max_distance:
            diff = max_distance - self.total_distance
            pos = Point3(camera_xy + self.direction_xy * diff, self.camera_z)
            self.camera.set_pos(pos)
            self.total_distance = 0
            return True

        pos = Point3(camera_xy + self.direction_xy * distance, self.camera_z)
        self.camera.set_pos(pos)
        self.total_distance = total

    def find_next_position(self):
        try:
            passing_pts = self.walker_controller.trace_q.popleft()

            if len(self.trace_q) > 0:
                # turn back
                if self.trace_q[-1].almost_equal(passing_pts.end.xy, 0.1):
                    current_xy = self.trace_q.pop()
                else:
                    current_xy = self.trace_q[-1]
                    self.trace_q.append(passing_pts.start.xy)

                next_xy = self.trace_q[-1]
                self.direction_xy = Vec2(next_xy - current_xy).normalized()
                return True

        except IndexError:
            pass

    def update(self, dt):
        match self.state:

            case Status.STOP:
                if self.find_next_position():
                    self.state = Status.MOVE

            case Status.MOVE:
                if self.move(dt):
                    self.state = Status.STOP
                self.camera.look_at(self.floater)


class MazeLand(ShowBase):

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

        self.camLens.set_fov(90)
        self.camLens.set_near_far(0.5, 100000)
        self.camera.reparent_to(self.render)
        self.camera_controller = CameraController(self.camera, self.walker_controller)
        self.camera_controller.setup_camera()

        # ############### aircraft part ############################
        self.aircraft_controller = AircraftController(self.world, self.maze_builder)

        # self.floater = NodePath('floater')
        # self.floater.set_z(5)
        # self.floater.reparent_to(self.aircraft_controller.aircraft.root_np)

        # self.camera.reparent_to(self.aircraft_controller.aircraft.root_np)
        # self.camera.set_pos(self.aircraft_controller.get_relative_pos(Point3(0, -2, 5)))  # Point3(0, -8, 6)
        # self.camera.look_at(self.aircraft_controller.aircraft.air_frame)
        # # self.camera.look_at(self.floater)
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
        pos = self.aircraft_controller.get_relative_pos(Point3(0, -2, 5))  # 5
        cam1 = self.create_cameras((0.0, 0.5, 0.0, 1), pos)
        cam1.reparent_to(self.aircraft_controller.aircraft.root_np)
        cam1.look_at(self.aircraft_controller.aircraft.air_frame)

        pos = self.walker_controller.navigate(Point3(0, 1, 2))
        cam2 = self.create_cameras((0.5, 1.0, 0, 1), pos)
        floater = NodePath('floater_2')
        floater.set_z(0.5)   # 3
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
        pass
        # # region = self.win.make_display_region((0., 0.25, 0.75, 1)) # 左上
        # region = self.win.make_display_region((0.75, 1, 0.75, 1)) # 右上
        # region.set_sort(100)
        # cam_np = NodePath(Camera('cam'))
        # cam_np.node().get_lens().set_aspect_ratio(3.0 / 4.0)
        # cam_np.node().get_lens().set_fov(90)
        # region.set_camera(cam_np)

        # cam_np.reparent_to(self.aircraft_controller.aircraft.root_np)
        # cam_np.set_pos(self.aircraft_controller.get_relative_pos(Point3(0, -2, 5)))  # Point3(0, -8, 6)
        # cam_np.look_at(self.aircraft_controller.aircraft.air_frame)

        # dr = self.win.make_display_region((0., 0.25, 0.75, 1))
        # dr.set_sort(20)
        # cam_np2 = NodePath(Camera('cam2'))
        # lens = OrthographicLens()
        # lens.set_film_size(2, 2)
        # lens.set_near_far(-1000, 1000)
        # cam_np2.node().set_lens(lens)

        # my_render2d = NodePath('myrender2d')
        # my_render2d.set_depth_test(False)
        # my_render2d.set_depth_write(False)
        # cam_np2.reparent_to(my_render2d)
        # dr.set_camera(cam_np2)

        # from direct.gui.DirectGui import OnscreenText
        # OnscreenText(
        #     text='Hello',
        #     parent=my_render2d,
        #     # align=TextNode.ALeft,
        #     # pos=(0.05, start_y - (i * 0.05)),
        #     fg=(1, 1, 1, 1),
        #     scale=0.5,
        # )

        # region = self.win.make_display_region((0., 0.25, 0.75, 1)) # 左上
        # # region = self.win.make_display_region((0.75, 1, 0.75, 1)) # 右上
        # region.set_sort(200)
        # cam_np = NodePath(Camera('cam'))
        # cam_np.node().get_lens().set_aspect_ratio(3.0 / 4.0)
        # cam_np.node().get_lens().set_fov(90)
        # region.set_camera(cam_np)

        # cam_np.reparent_to(self.aircraft_controller.aircraft.root_np)
        # cam_np.set_pos(self.aircraft_controller.get_relative_pos(Point3(0, -2, 5)))  # Point3(0, -8, 6)
        # cam_np.look_at(self.aircraft_controller.aircraft.air_frame)

    def toggle_debug(self):
        if self.debug.is_hidden():
            self.debug.show()
        else:
            self.debug.hide()

    def print_info(self):
        self.camera.look_at(self.walker_controller.walker.character)
        walker_pos = self.walker_controller.get_walker_pos()
        print('walker_pos', walker_pos)
        # print('camera_pos', self.camera.get_pos(self.render))
        # print('backward_pos', self.walker_controller.navigate(Vec3(0, 2, 3)) + walker_pos)
        # print('camera reative pos', self.camera.get_pos())
        # print('relative_pos', self.walker_controller.navigate(Vec3(0, -1, 0)))
        # print('forwad_vector', self.walker_controller.walker.direction_np.get_quat(base.render).get_forward())
        # print('camera_forwad_vector', self.camera.get_quat(base.render).get_forward())

    def get_key_input(self):
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

        return direction

    def update(self, task):
        dt = globalClock.get_dt()
        direction = self.get_key_input()
        self.walker_controller.update(direction, dt)

        match self.walker_controller.state:

            case Status.DO_JUMP:
                self.camera_controller.change_z()

            case _:
                self.camera_controller.update(dt)

        # self.aircraft_controller.update(dt)
        self.world.do_physics(dt)
        return task.cont


if __name__ == '__main__':
    app = MazeLand()
    app.run()
