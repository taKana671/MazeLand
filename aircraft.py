import random

from panda3d.bullet import BulletRigidBodyNode, BulletSphereShape, BulletConvexHullShape
from panda3d.core import NodePath
from panda3d.core import TransformState, Vec3, BitMask32, Point3, LColor, Vec2

from create_geomnode import SphericalShape, RightTriangularPrism
from utils import create_line_node
from basic_character import Status, Direction, Sensor, BodyColor


class AirFrame(NodePath):

    def __init__(self, body_color):
        super().__init__(BulletRigidBodyNode(body_color.name.lower()))
        # self.set_collide_mask(BitMask32.bit(2))
        self.set_collide_mask(BitMask32.bit(5))
        self.node().set_kinematic(True)
        self.node().set_ccd_motion_threshold(1e-7)
        self.node().set_ccd_swept_sphere_radius(0.5)

        self.create_body(body_color.value)
        self.create_wings()

    def create_body(self, body_color):
        body = SphericalShape(radius=0.5)
        body.set_scale(Vec3(1))  # if oval, set Vec3(1, 1.5, 1).
        # body.set_scale(Vec3(1, 1.5, 1))
        body.set_color(body_color)
        body.reparent_to(self)

        end, tip = body.get_tight_bounds()
        shape = BulletSphereShape((tip - end).z / 2)
        self.node().add_shape(shape, TransformState.make_pos(Point3(0, 0, 0)))

    def create_wings(self):
        wing = RightTriangularPrism(h=0.2)
        pos = Vec3(0, -0.3, 0)
        # pos = Vec3(0, -0.5, 0)
        hpr = Vec3(-135, 0, 0)
        wing.set_pos_hpr(pos, hpr)  # if oval: set pos to Vec3(0, -0.5, 0).
        wing.set_color(LColor(.5, .5, .5, 1))
        wing.reparent_to(self)

        shape = BulletConvexHullShape()
        shape.add_geom(wing.node().get_geom(0))
        self.node().add_shape(shape, TransformState.make_pos_hpr(pos, hpr))


class Aircraft:

    def __init__(self, world, body_color, linear_velocity=5, angular_velocity=100):
        self.world = world
        self.root_np = NodePath('root')
        self.direction_np = NodePath('direction')
        self.air_frame = AirFrame(body_color)
        self.air_frame.reparent_to(self.direction_np)
        self.direction_np.reparent_to(self.root_np)
        self.root_np.reparent_to(base.render)
        self.world.attach(self.air_frame.node())

        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.climbing_velocity = 10

        self.total_distance = 0
        self.total_angle = 0
        self.total = 0

        self.create_sensors()
        self.draw_debug_lines()

    def set_pos(self, pos):
        self.root_np.set_pos(pos)

    def get_pos(self):
        return self.root_np.get_pos()

    def get_backward_pos(self, distance):
        """Return backward position
           Args:
               distance(float): must be positive.
        """
        backward_vector = self.direction_np.get_quat(base.render).get_forward() * -1
        return self.get_pos() + backward_vector * distance

    def create_sensors(self):
        self.sensors = [
            Sensor(self.world, Direction.FORWARD),
            Sensor(self.world, Direction.LEFTWARD),
            Sensor(self.world, Direction.RIGHTWARD),
        ]

        for sensor in self.sensors:
            sensor.reparent_to(self.direction_np)

    def draw_debug_lines(self):
        color = LColor(0, 0, 1, 1)

        for sensor in self.sensors:
            line = create_line_node(
                pos := self.direction_np.get_pos(self.root_np),
                pos + sensor.get_pos(self.root_np),
                color
            )
            line.reparent_to(self.direction_np)

    def detect_route(self):
        pos_from = self.get_pos()

        for sensor in self.sensors:
            if not sensor.detect_obstacles(pos_from):
                yield sensor.direction

    def turn(self, rotate_direction, dt, max_angle=90):
        angle = self.angular_velocity * dt

        if (total := self.total_angle + angle) >= max_angle:
            diff = max_angle - self.total_angle
            self.direction_np.set_h(self.direction_np.get_h() - diff * rotate_direction)
            self.total_angle = 0
            return True

        self.total_angle = total
        self.direction_np.set_h(self.direction_np.get_h() - angle * rotate_direction)

    # def move_forward(self, max_distance, dt):
    def move_forward(self, dt, max_distance=2):
        forward_vector = self.direction_np.get_quat(base.render).get_forward()
        distance = self.linear_velocity * dt
        self.total_distance += distance

        if (diff := self.total_distance - max_distance) >= 0:
            self.set_pos(self.get_pos() + forward_vector * (distance - diff))
            self.total_distance = 0
            return True

        self.set_pos(self.get_pos() + forward_vector * distance)

    def check_forward(self, other, bit):
        sensor = self.sensors[0]
        pos_from = self.get_pos()

        if result := sensor.detect_obstacles(pos_from, bit):
            if result.get_node() == other.aircraft.air_frame.node():
                return True

    def check_downward(self, bit=5):
        current_pos = self.root_np.get_pos()
        current_pos.z -= 0.5
        ts_from = TransformState.make_pos(current_pos)
        ts_to = TransformState.make_pos(current_pos - Vec3(0, 0, 5))
        shape = BulletSphereShape(0.5)

        if (result := self.world.sweep_test_closest(
                shape, ts_from, ts_to, BitMask32.bit(5), 0.0)).has_hit():
            
            print(result.get_node().get_name())
            return True

    def lift(self, dt, max_distance=1, direction=1):
        distance = self.climbing_velocity * dt
        self.total += distance

        if (total := self.total + distance) >= max_distance:
            z = self.root_np.get_z() + (max_distance - self.total) * direction
            self.root_np.set_z(z)
            self.total = 0
            return True

        self.root_np.set_z(self.root_np.get_z() + distance * direction)
        self.total = total


class Controller:

    # def __init__(self, world, maze_builder):
    def __init__(self, world, maze_builder, body_color, start_xy):
        self.world = world
        self.maze = maze_builder
        self.dead_end = None
        self.stop = False
        self.others = []

        self.aircraft = Aircraft(self.world, body_color)
        z = self.maze.wall_size.z - 0.5
        maze_z = self.maze.get_maze_pos().z
        self.aircraft_z = z + maze_z

        self.aircraft.set_pos(Point3(start_xy, self.aircraft_z))
        print('aircraft pos', Point3(start_xy, self.aircraft_z))
      
        

        self.state = None

        # self.aircraft_2 = Aircraft(self.world, LColor(1, 0, 0, 1), 2)
        # xy = self.maze.get_entrance() + Vec2(0, -2)
        # z = self.maze.wall_size.z - 0.5
        # maze_z = self.maze.get_maze_pos().z
        # self.aircraft_2.set_pos(Point3(xy, z + maze_z))

    def get_relative_pos(self, pos):
        """Return a relative point to enable camera to follow the character
           when the camera's view is blocked by an object like wall.
        """
        return self.aircraft.root_np.get_relative_point(self.aircraft.direction_np, pos)

    @property
    def aircraft_pos(self):
        return self.aircraft.root_np.get_pos()

    def close_route(self):
        pos = self.aircraft.get_backward_pos(2.)
        scale = Vec3(self.maze.wall_size.xy, 1)
        mask = BitMask32.bit(2)
        self.maze.make_block('closed', pos, scale, mask, True)

    def change_direction(self):
        if not (directions := [d for d in self.aircraft.detect_route()]):
            self.dead_end = True
            return Status.U_TURN

        if len(directions) >= 2:
            if self.dead_end:
                self.close_route()
                self.dead_end = False
            random.shuffle(directions)

        direction = directions[0]

        match direction:
            case Direction.FORWARD:

                # for other in self.others:
                #     if self.aircraft.check_forward(other, 5):
                #         if other.state in (Status.LEFT_TURN, Status.RIGHT_TURN):
                #             return Status.WAIT
                #         elif other.state == Status.U_TURN:
                #             return Status.U_TURN
                #         if other.state == Status.MOVE:
                #             print(
                #                 self.aircraft.direction_np.get_quat(base.render).get_forward(),
                #                 other.aircraft.direction_np.get_quat(base.render).get_forward()
                #             )

                return Status.MOVE

            case Direction.LEFTWARD:
                return Status.LEFT_TURN

            case Direction.RIGHTWARD:
                return Status.RIGHT_TURN

    def update(self, dt):
        if not self.stop:
            match self.state:

                case Status.STOP:
                    self.state = self.change_direction()

                case Status.MOVE:
                    # if self.aircraft.move_forward(2, dt):
                    if self.aircraft.move_forward(dt):
                        if self.maze.is_outside(self.aircraft_pos.xy):
                            self.state = Status.FINISH
                            base.messenger.send('finish')

                        elif self.aircraft.root_np.get_z() > self.aircraft_z:
                            self.state = Status.CHECK_DOWNWARD

                        else:
                            self.state = Status.STOP

                case Status.LEFT_TURN:
                    if self.aircraft.turn(Direction.LEFTWARD.get_direction(), dt):
                        self.state = Status.MOVE

                case Status.RIGHT_TURN:
                    if self.aircraft.turn(Direction.RIGHTWARD.get_direction(), dt):
                        self.state = Status.MOVE

                case Status.U_TURN:
                    if self.aircraft.turn(Direction.LEFTWARD.get_direction(), dt, 180):
                        self.state = Status.MOVE

                case Status.LIFT_UP:
                    if self.aircraft.lift(dt):
                        self.state = Status.MOVE

                case Status.LIFT_DOWN:
                    if self.aircraft.lift(dt, direction=-1):
                        self.state = Status.STOP

                case Status.CHECK_DOWNWARD:
                    if not self.aircraft.check_downward():
                        self.state = Status.LIFT_DOWN

    def start(self):
        self.state = Status.STOP


class AircraftController:

    def __init__(self, world, maze_builder):
        self.world = world
        self.maze = maze_builder

        temp_xy = self.maze.get_entrance() + Vec2(0, 2)
        xy = temp_xy
        self.controller_1 = Controller(self.world, self.maze, BodyColor.BLUE, xy)
        xy = temp_xy + Vec2(0, -2)
        self.controller_2 = Controller(self.world, self.maze, BodyColor.RED, xy)
        self.state = None

    
    def detect_accident(self):
        aircraft1_nd = self.controller_1.aircraft.air_frame.node()
        aircraft2_nd = self.controller_2.aircraft.air_frame.node()

        if (result := self.world.contact_test_pair(
                aircraft1_nd, aircraft2_nd)).get_num_contacts() > 0:
            return result

    def update(self, dt):
        self.controller_1.update(dt)
        self.controller_2.update(dt)

        match self.state:
            case Status.PLAY:
                if self.detect_accident():
                    if self.controller_1.state == Status.MOVE:
                        self.controller_1.state = Status.LIFT_UP
                        self.target_aircraft = self.controller_1
                        if self.controller_2.state == Status.MOVE:
                            self.controller_2.stop = True
                    elif self.controller_2.state == Status.MOVE:
                        self.controller_2.state = Status.LIFT_UP
                        self.target_aircraft = self.controller_2
                        if self.controller_1.state == Status.MOVE:
                            self.controller_1.stop = True

                    self.state = Status.WAIT

            case Status.WAIT:
                if self.target_aircraft.state == Status.MOVE:
                    if self.target_aircraft == self.controller_1:
                        self.controller_2.stop = False
                    else:
                        self.controller_1.stop = False

    def start(self):
        self.state = Status.PLAY
        self.controller_1.start()
        self.controller_2.start()
