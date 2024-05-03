import random
from enum import Enum, StrEnum, auto

from panda3d.bullet import BulletRigidBodyNode, BulletSphereShape, BulletConvexHullShape
from panda3d.core import NodePath, PandaNode
from panda3d.core import TransformState, Vec3, BitMask32, Point3, LColor

from create_geomnode import SphericalShape, RightTriangularPrism
from utils import create_line_node


class Status(Enum):

    MOVE = auto()
    STOP = auto()
    LEFT_TURN = auto()
    RIGHT_TURN = auto()
    U_TURN = auto()
    GO_BACK = auto()
    CHECK_ROUTE = auto()


class Direction(StrEnum):

    FORWARD = auto()
    RIGHT = auto()
    LEFT = auto()


class AirFrame(NodePath):

    def __init__(self):
        super().__init__(BulletRigidBodyNode('air_frame'))
        self.set_collide_mask(BitMask32.bit(1))
        self.node().set_kinematic(True)
        self.node().set_ccd_motion_threshold(1e-7)
        self.node().set_ccd_swept_sphere_radius(0.5)

        self.create_body()
        self.create_wings()

    def create_body(self):
        body = SphericalShape(radius=0.5)
        body.set_scale(Vec3(1))  # if oval, set Vec3(1, 1.5, 1).
        body.set_color(LColor(0, 0, 1, 1))
        body.reparent_to(self)

        end, tip = body.get_tight_bounds()
        shape = BulletSphereShape((tip - end).z / 2)
        self.node().add_shape(shape, TransformState.make_pos(Point3(0, 0, 0)))

    def create_wings(self):
        wing = RightTriangularPrism(h=0.2)
        pos = Vec3(0, -0.3, 0)
        hpr = Vec3(-135, 0, 0)
        wing.set_pos_hpr(pos, hpr)  # if oval: set pos to Vec3(0, -0.5, 0).
        wing.set_color(LColor(.5, .5, .5, 1))
        wing.reparent_to(self)

        shape = BulletConvexHullShape()
        shape.add_geom(wing.node().get_geom(0))
        self.node().add_shape(shape, TransformState.make_pos_hpr(pos, hpr))


class Sensor(NodePath):

    def __init__(self, direction, pos, world):
        super().__init__(PandaNode(direction.value))
        self.world = world
        self.direction = direction
        self.set_pos(pos)

    def detect_obstacles(self, pos_from, bit=2):
        pos_to = self.get_pos(base.render)

        if (result := self.world.ray_test_closest(
                pos_from, pos_to, mask=BitMask32.bit(bit))).has_hit():
            return result


class Aircraft:

    def __init__(self, world, linear_velocity=5, angular_velocity=100):
        self.world = world
        self.root_np = NodePath('root')
        self.direction_np = NodePath('direction')
        air_frame = AirFrame()
        air_frame.reparent_to(self.direction_np)
        self.direction_np.reparent_to(self.root_np)
        self.root_np.reparent_to(base.render)
        self.world.attach(air_frame.node())

        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

        self.total_distance = 0
        self.total_angle = 0

        self.create_sensors()
        # self.draw_debug_lines()

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
            Sensor(Direction.FORWARD, Vec3(0, 1.5, 0), self.world),
            Sensor(Direction.LEFT, Vec3(-1.5, 0, 0), self.world),
            Sensor(Direction.RIGHT, Vec3(1.5, 0, 0), self.world),
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

    def turn_right(self, max_angle, dt):
        angle = -self.angular_velocity * dt
        self.total_angle += angle

        if (diff := self.total_angle - max_angle) <= 0:
            self.direction_np.set_h(self.direction_np.get_h() + angle - diff)
            self.total_angle = 0
            return True

        self.direction_np.set_h(self.direction_np.get_h() + angle)

    def turn_left(self, max_angle, dt):
        angle = self.angular_velocity * dt
        self.total_angle += angle

        if (diff := self.total_angle - max_angle) >= 0:
            self.direction_np.set_h(self.direction_np.get_h() + angle - diff)
            self.total_angle = 0
            return True

        self.direction_np.set_h(self.direction_np.get_h() + angle)

    def move_forward(self, max_distance, dt):
        forward_vector = self.direction_np.get_quat(base.render).get_forward()
        distance = self.linear_velocity * dt
        self.total_distance += distance

        if (diff := self.total_distance - max_distance) >= 0:
            self.set_pos(self.get_pos() + forward_vector * (distance - diff))
            self.total_distance = 0
            return True

        self.set_pos(self.get_pos() + forward_vector * distance)


class AircraftController:

    def __init__(self, world, maze_builder):
        self.world = world
        self.maze_builder = maze_builder
        self.dead_end = None

        self.aircraft = Aircraft(self.world)
        xy = self.maze_builder.get_entrance()
        z = self.maze_builder.wall_wd.x * 2 - 0.5
        self.aircraft.set_pos(Point3(xy, z - 10))

        self.state = Status.STOP

    def get_relative_pos(self, pos):
        """Return a relative point to enable camera to follow the character
           when the camera's view is blocked by an object like wall.
        """
        return self.aircraft.root_np.get_relative_point(self.aircraft.direction_np, pos)

    def close_route(self):
        pos = self.aircraft.get_backward_pos(2.)
        scale = Vec3(self.maze_builder.wall_wd.xy, 1)
        mask = BitMask32.bit(1) | BitMask32.bit(2)
        self.maze_builder.make_block('closed', pos, scale, mask, True)

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
                return Status.MOVE

            case Direction.LEFT:
                return Status.LEFT_TURN

            case Direction.RIGHT:
                return Status.RIGHT_TURN

    def update(self, dt):
        match self.state:

            case Status.STOP:
                self.state = self.change_direction()

            case Status.MOVE:
                if self.aircraft.move_forward(2, dt):
                    self.state = Status.STOP

            case Status.LEFT_TURN:
                if self.aircraft.turn_left(90, dt):
                    self.state = Status.MOVE

            case Status.RIGHT_TURN:
                if self.aircraft.turn_right(-90, dt):
                    self.state = Status.MOVE

            case Status.U_TURN:
                if self.aircraft.turn_left(180, dt):
                    self.state = Status.MOVE