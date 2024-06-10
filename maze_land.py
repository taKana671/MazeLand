import math
import sys
from collections import deque

from panda3d.bullet import BulletWorld, BulletDebugNode
from panda3d.core import Vec3, NodePath, Point3, LColor, Vec2, Vec4
from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
from direct.showbase.InputStateGlobal import inputState
from panda3d.core import load_prc_file_data
from panda3d.core import Camera

from scene import Scene
from aircraft import Aircraft
from maze_walker import MazeWalker
from basic_character import Direction, Status, BodyColor
from screen import Screen, Button, Frame, Label


load_prc_file_data("", """
    textures-power-2 none
    gl-coordinate-system default
    window-title Panda3D Avoid Balls
    filled-wireframe-apply-shader true
    stm-max-views 8
    stm-max-chunk-count 2048""")


class CameraController:

    def __init__(self, camera, walker_q, floater):
    # def __init__(self, walker_q, floater_parent):
        self.walker_q = walker_q

        self.floater = floater
        self.camera = camera

        # self.floater = NodePath('floater')
        # self.floater.set_z(1)   # 3
        # self.floater.reparent_to(floater_parent)

        self.state = Status.STOP
        self.total_distance = 0
        self.trace_q = deque()

    def get_camera_z(self, walker_pos):
        return walker_pos.z + 1

    def setup(self, pos):
        self.trace_q.append(pos.xy)
        self.camera.set_pos(pos)
        self.camera.look_at(self.floater)

    def setup_camera(self, camera, pos):
        self.camera = camera
        self.trace_q.append(pos.xy)
        self.camera.set_pos(pos)
        self.camera.look_at(self.floater)

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
                z = self.get_camera_z(walker_pos)
                self.camera.set_z(z)

            case Status.CRASH:
                self.camera.look_at(self.floater)

            case _:
                self.follow(dt, walker_pos)


class MazeLand(ShowBase):

    def __init__(self):
        super().__init__()
        self.set_background_color(LColor(1, 1, 1, 1))
        self.disable_mouse()

        self.world = BulletWorld()
        self.world.set_gravity(Vec3(0, 0, -9.81))
        self.scene = Scene(self.world)
        self.scene.build_maze()

        self.aircraft_1 = Aircraft(self.world, self.scene.maze, BodyColor.BLUE, Vec2(0, -2))
        self.aircraft_2 = Aircraft(self.world, self.scene.maze, BodyColor.RED, Vec2(0, -2) * 2)

        self.walker_q = deque()
        self.walker = MazeWalker(self.world, self.scene.maze, self.walker_q)
        self.floater = NodePath('floater')
        self.floater.set_z(1)   # 3
        self.floater.reparent_to(self.walker.body)

        self.camera_controller = CameraController(self.walker_q, self.walker.body)

        # self.create_display_regions()
        self.split_screen()
        self.create_gui()

        self.accident_aircrafts = []
        self.walker_state = None
        self.aircrafts_state = None
        self.state = None

        self.debug = self.render.attach_new_node(BulletDebugNode('debug'))
        self.world.set_debug_node(self.debug.node())

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

        self.again_frame = Frame(self.aspect2d)
        Label(self.again_frame, 'Try Again', (0, 0, 0.3), font)
        Button(self.again_frame, 'START', (0, 0, 0), font, command=lambda: self.screen.fade_out(self.start_again), focus=True)
        Button(self.again_frame, 'EXIT', (0, 0, -0.2), font, command=sys.exit)
        self.again_frame.hide()

        start_frame = Frame(self.aspect2d)
        Label(start_frame, 'Maze Land', (0, 0, 0.3), font)
        Button(start_frame, 'START', (0, 0, 0), font, command=lambda: self.screen.fade_out(self.start_game), focus=True)
        Button(start_frame, 'EXIT', (0, 0, -0.2), font, command=sys.exit)

        self.screen = Screen(start_frame)
        self.screen.show()

    def finish(self):
        def _finish():
            self.accept('escape', sys.exit)
            for obj in [self.walker, self.aircraft_1, self.aircraft_2]:
                obj.state = None

        self.screen.frame = self.again_frame
        self.screen.fade_in(_finish)
        # self.screen.fade_in(self.accept, 'escape', sys.exit)


    def initialize(self):
        self.walker_q.clear()
        self.accident_aircrafts.clear()
        self.walker_state = None
        self.aircrafts_state = None
        self.state = None

        for player in [self.walker, self.aircraft_1, self.aircraft_2]:
            player.initialize()



    def start_again(self):
        self.scene.destroy_maze()
        self.state = Status.CLEAN_UP

    def start_game(self):
        self.accept('escape', sys.exit)
        self.walker.start()
        # self.region_l.set_sort(100)
        self.aircrafts_state = Status.READY
        self.walker_state = Status.PLAY

    def calc_aspect_ratio(self, window_size, display_region):
        """Return aspect ratio.
            Args:
                window_size (Vec2): current window size; Vec2(width, height)
                display_region (Vec4): (left, right, bottom, top); The ranges are from 0 to 1,
                               where 0 is the left and bottomof the window,
                               and 1 is the right and top of the window.
        """
        region_w = display_region.y - display_region.x
        region_h = display_region.w - display_region.z
        display_w = int(window_size.x * region_w)
        display_h = int(window_size.y * region_h)

        gcd = math.gcd(display_w, display_h)
        w = display_w / gcd
        h = display_h / gcd
        aspect_ratio = w / h

        return aspect_ratio

    def split_screen(self):
        props = self.win.get_properties()
        window_size = props.get_size()
        rel_y = self.scene.maze.wall_size.y

        # make split screen for aircrafts
        rel_pos = Point3(0, -rel_y, 5)

        aircraft_regions = [
            [self.aircraft_1, Vec4(0., 0.499, 0.75, 1)],  # left top; Vec4(0., 0.5, 0.75, 1); 0.5 - 0.001 to make white line.
            [self.aircraft_2, Vec4(0.501, 1, 0.75, 1)]    # right top; Vec4(0.5., 1, 0.75, 1); 0.5 + 0.001 to make white line.
        ]

        for aircraft, region in aircraft_regions:
            pos = aircraft.get_relative_pos(rel_pos)
            cam = self.create_split_screen_camera(region, window_size)
            cam.set_pos(pos)
            cam.reparent_to(aircraft.root_np)
            cam.look_at(aircraft.body)

        # make split screen walker
        cam_pos = self.walker.navigate(Point3(0, rel_y, 1)) + self.walker.get_pos()

        region = Vec4(0, 1, 0, 0.748)  # Vec4(0., 0.5, 0.75, 1); 0.75 - 0.002 to make white line.
        cam = self.create_split_screen_camera(region, window_size, near=0.5)
        cam.reparent_to(self.render)

        # self.camera_controller = CameraController(cam, self.walker_q, self.floater)

        self.camera_controller.setup_camera(cam, cam_pos)

        self.camNode.set_active(False)

    def create_split_screen_camera(self, region, window_size, fov=90, near=1, far=100000):
        """Create a camera for split screen.
            Args:
                region (Vec4): display region; left, right, bottom, top
        """
        camera = self.make_camera(self.win, displayRegion=region)
        aspect_ratio = self.calc_aspect_ratio(window_size, region)
        camera.node().get_lens().set_aspect_ratio(aspect_ratio)
        camera.node().get_lens().set_fov(fov)
        camera.node().get_lens().set_near_far(near, far)

        return camera

    def create_display_regions(self):
        props = self.win.get_properties()
        window_size = props.get_size()
        rel_y = self.scene.maze.wall_size.y

        # make split screen for aircrafts
        rel_pos = Point3(0, -rel_y, 5)

        aircraft_regions = [
            [self.aircraft_1, Vec4(0., 0.499, 0.75, 1)],  # left top; Vec4(0., 0.5, 0.75, 1); 0.5 - 0.001 to make white line.
            [self.aircraft_2, Vec4(0.501, 1, 0.75, 1)]    # right top; Vec4(0.5., 1, 0.75, 1); 0.5 + 0.001 to make white line.
        ]

        for i, (aircraft, region) in enumerate(aircraft_regions):
            display_region = self.win.make_display_region(region)
            cam = self.create_region_camera(f'cam_aircraft_{i}', region, window_size)
            # needs set_sort if one region overlaps another.
            # display_region.set_sort(100 + i)
            display_region.set_camera(cam)
            cam.set_pos(aircraft.get_relative_pos(rel_pos))  # Point3(0, -8, 6)
            cam.reparent_to(aircraft.root_np)
            cam.look_at(aircraft.body)

        cam_pos = self.walker.navigate(Point3(0, rel_y, 1)) + self.walker.get_pos()
        region = Vec4(0, 1, 0, 0.748)  # Vec4(0., 0.5, 0.75, 1); 0.75 - 0.002 to make white line.
        display_region = self.win.make_display_region(region)
        cam = self.create_region_camera('cam_walker', region, window_size, near=0.5)
        display_region.set_camera(cam)
        cam.reparent_to(self.render)
        self.camera_controller.setup_camera(cam, cam_pos)

        self.camNode.set_active(False)

    def create_region_camera(self, name, region, window_size, fov=90, near=1, far=100000):
        camera_np = NodePath(Camera(name))
        aspect_ratio = self.calc_aspect_ratio(window_size, region)
        camera_np.node().get_lens().set_aspect_ratio(aspect_ratio)
        camera_np.node().get_lens().set_fov(fov)
        camera_np.node().get_lens().set_near_far(near, far)

        return camera_np

    def toggle_debug(self):
        if self.debug.is_hidden():
            self.debug.show()
        else:
            self.debug.hide()

    def print_info(self):
        print('walker_pos', self.walker_controller.walker_pos)

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

    def control_walker(self, dt):
        direction = self.get_key_input()
        self.walker.update(direction, dt)
        self.camera_controller.update(dt, self.walker.get_pos(), self.walker.state)

        match self.walker_state:

            case Status.PLAY:
                for aircraft in [self.aircraft_1, self.aircraft_2]:
                    if aircraft.detect_collision(self.walker.body):
                        self.accident_aircrafts.append(aircraft)
                        aircraft.stop = True

                if self.accident_aircrafts:
                    # print(self.accident_aircrafts)
                    self.walker.crash()
                    self.walker_state = Status.WAIT

            case Status.WAIT:
                for aircraft in self.accident_aircrafts:
                    if not aircraft.detect_collision(self.walker.body):
                        aircraft.stop = False

                if all(not aircraft.stop for aircraft in self.accident_aircrafts):
                    self.accident_aircrafts.clear()
                    self.walker_state = Status.PLAY

    def control_aircrafts(self, dt):
        self.aircraft_1.update(dt)
        self.aircraft_2.update(dt)

        match self.aircrafts_state:

            case Status.PLAY:
                if self.aircraft_1.detect_collision(self.aircraft_2.body):
                    ascend = False

                    for aircraft in [self.aircraft_1, self.aircraft_2]:
                        aircraft.dead_end = False

                        if aircraft.state == Status.MOVE:
                            if not ascend:
                                aircraft.state = Status.LIFT_UP
                                ascend = True
                                continue
                            aircraft.stop = True

                    self.aircrafts_state = Status.WAIT

            case Status.WAIT:
                if not self.aircraft_1.detect_collision(self.aircraft_2.body):
                    for aircraft in [self.aircraft_1, self.aircraft_2]:
                        if aircraft.stop:
                            aircraft.stop = False
                    self.aircrafts_state = Status.PLAY

            case Status.READY:
                self.aircraft_1.start(0.5)
                self.aircraft_2.start(1)

                self.aircrafts_state = Status.PLAY

    def update(self, task):
        dt = globalClock.get_dt()
        self.control_aircrafts(dt)
        self.control_walker(dt)

        match self.state:
            case Status.CLEAN_UP:
                self.state = Status.STOP

            case Status.STOP:
                self.scene.build_maze()
                xy = self.scene.maze.get_entrance()
                xy += Vec2(0, 2)
                self.walker.root_np.set_pos(Point3(xy, -8.5))
                self.state = None

        self.world.do_physics(dt)
        return task.cont


if __name__ == '__main__':
    app = MazeLand()
    app.run()
