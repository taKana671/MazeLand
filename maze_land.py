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

from lights import BasicAmbientLight, BasicDayLight
from scene import Scene
from aircraft import AircraftController
from maze_walker import MazeWalkerController
from basic_character import Direction, Status
from screen import Screen, Button, Frame, Label


load_prc_file_data("", """
    textures-power-2 none
    gl-coordinate-system default
    window-title Panda3D Avoid Balls
    filled-wireframe-apply-shader true
    stm-max-views 8
    stm-max-chunk-count 2048""")


class CameraController:

    def __init__(self, camera, walker_q, floater_parent):
        self.camera = camera
        self.walker_q = walker_q
        self.before_walker_pos = None

        self.floater = NodePath('floater')
        self.floater.set_z(1)   # 3
        self.floater.reparent_to(floater_parent)

        self.state = Status.STOP
        self.total_distance = 0
        self.trace_q = deque()

    def get_camera_z(self, walker_pos):
        return walker_pos.z + 1

    def setup_camera(self, pos):
        self.trace_q.append(pos.xy)
        self.camera.set_pos(pos)
        self.camera.look_at(self.floater)

    def change_z(self, walker_pos):
        if self.before_walker_pos != walker_pos:
            z = self.get_camera_z(walker_pos)
            self.camera.set_z(z)

    def move(self, dt, walker_pos, max_distance=2):
        distance = dt * 2
        camera_xy = self.camera.get_pos().xy
        camera_z = self.get_camera_z(walker_pos)

        if (total := self.total_distance + distance) >= max_distance:
            diff = max_distance - self.total_distance
            pos = Point3(camera_xy + self.direction_xy * diff, camera_z)
            self.camera.set_pos(pos)
            self.total_distance = 0
            return True

        pos = Point3(camera_xy + self.direction_xy * distance, camera_z)
        self.camera.set_pos(pos)
        self.total_distance = total

    def find_next_position(self):
        try:
            passing_pts = self.walker_q.popleft()

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

    def follow(self, dt, walker_pos):
        match self.state:

            case Status.STOP:
                if self.find_next_position():
                    self.state = Status.MOVE

            case Status.MOVE:
                if self.move(dt, walker_pos):
                    self.state = Status.STOP
                self.camera.look_at(self.floater)

    def update(self, dt, walker_pos, walker_state):

        match walker_state:

            case Status.DO_JUMP:
                self.change_z(walker_pos)

            case Status.CRASH:
                self.camera.look_at(self.floater)

            case _:
                self.follow(dt, walker_pos)


class MazeLand(ShowBase):  

    def __init__(self):
        super().__init__()
        self.disable_mouse()
        self.world = BulletWorld()
        self.world.set_gravity(Vec3(0, 0, -9.81))

        self.debug = self.render.attach_new_node(BulletDebugNode('debug'))
        self.world.set_debug_node(self.debug.node())

        self.scene = Scene(self.world)

        self.aircraft_controller = AircraftController(self.world, self.scene.maze)
        walker_q = deque()
        self.walker_controller = MazeWalkerController(self.world, self.scene.maze, walker_q)

        self.camLens.set_fov(90)
        self.camLens.set_near_far(0.5, 100000)
        self.camera.reparent_to(self.render)
        self.camera_controller = CameraController(self.camera, walker_q, self.walker_controller.walker.character)

        camera_pos = self.walker_controller.navigate(Point3(0, 2.0, 1)) + self.walker_controller.walker_pos
        self.camera_controller.setup_camera(camera_pos)

        self.state = None
 
        # ############### aircraft part ############################
        # self.camera.reparent_to(self.aircraft_controller.aircraft.root_np)
        # self.camera.set_pos(self.aircraft_controller.get_relative_pos(Point3(0, -2, 5)))  # Point3(0, -8, 6)
        # self.camera.look_at(self.aircraft_controller.aircraft.air_frame)
        # ###########################################

        self.ambient_light = BasicAmbientLight()
        self.ambient_light.reparent_to(self.render)
        self.day_light = BasicDayLight()
        self.day_light.reparent_to(self.render)

        self.create_display_regions()
        # self.split_screen()
        self.create_gui()

        inputState.watch_with_modifiers('forward', 'arrow_up')
        inputState.watch_with_modifiers('backward', 'arrow_down')
        inputState.watch_with_modifiers('left', 'arrow_left')
        inputState.watch_with_modifiers('right', 'arrow_right')
        inputState.watch_with_modifiers('jump', 'enter')

        # self.accept('escape', sys.exit)
        self.accept('d', self.toggle_debug)
        self.accept('p', self.print_info)
        self.accept('finish', self.finish)
        self.taskMgr.add(self.update, 'update')

    def create_gui(self):
        font = self.loader.loadFont('font/Candaral.ttf')
        frame = Frame(self.aspect2d)
        self.screen = Screen(frame)

        Label(frame, 'Maze Land', (0, 0, 0.3), font)
        Button(frame, 'START', (0, 0, 0), font, command=lambda: self.screen.fade_out(self.start_game), focus=True)
        Button(frame, 'EXIT', (0, 0, -0.2), font, command=sys.exit)

        self.screen.show()

    def finish(self):
        self.state = None
        self.walker_controller.state = None
        self.aircraft_controller.state = None
        self.screen.fade_in(self.accept, 'escape', sys.exit)

    def start_game(self):
        self.accept('escape', sys.exit)
        self.walker_controller.start()
        self.aircraft_controller.start()
        # self.region_l.set_active(True)
        self.region_l.set_sort(100)
        self.state = Status.PLAY

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

    def create_display_regions(self):
        self.region_l = self.win.make_display_region((0., 0.25, 0.75, 1)) # 左上
        # region = self.win.make_display_region((0.75, 1, 0.75, 1)) # 右上
        # self.region_l.set_sort(100)
        self.region_l.set_sort(1)
        cam_np = NodePath(Camera('cam'))
        cam_np.node().get_lens().set_aspect_ratio(3.0 / 4.0)
        cam_np.node().get_lens().set_fov(90)
        self.region_l.set_camera(cam_np)

        cam_np.reparent_to(self.aircraft_controller.aircraft.root_np)
        cam_np.set_pos(self.aircraft_controller.get_relative_pos(Point3(0, -2, 5)))  # Point3(0, -8, 6)
        cam_np.look_at(self.aircraft_controller.aircraft.air_frame)
        # self.region_l.set_active(False)


        # main_region = self.win.get_display_region(0)
        # main_region.set_sort(0)


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
        self.state = Status.PLAY
        self.aircraft_controller.state = Status.STOP

        # self.camera.look_at(self.walker_controller.walker.character)
        print('walker_pos', self.walker_controller.walker_pos)

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

    def detect_accident(self):
        walker_nd = self.walker_controller.walker.character.node()
        aircraft_nd = self.aircraft_controller.aircraft.air_frame.node()

        if (result := self.world.contact_test_pair(
                walker_nd, aircraft_nd)).get_num_contacts() > 0:
            return result

    def update(self, task):
        dt = globalClock.get_dt()
        self.aircraft_controller.update(dt)
        direction = self.get_key_input()
        self.walker_controller.update(direction, dt)

        self.camera_controller.update(
            dt, self.walker_controller.walker_pos, self.walker_controller.state)

        match self.state:

            case Status.PLAY:
                if self.detect_accident():
                    self.walker_controller.crash(self.aircraft_controller.aircraft_pos)
                    self.aircraft_controller.stop = True
                    self.state = Status.WAIT

            case Status.WAIT:
                if not self.detect_accident():
                    self.aircraft_controller.stop = False

        self.world.do_physics(dt)
        return task.cont


if __name__ == '__main__':
    app = MazeLand()
    app.run()
