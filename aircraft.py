import random

from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletSphereShape, BulletConvexHullShape
from panda3d.core import NodePath
from panda3d.core import TransformState, Vec3, BitMask32, Point3, LColor, Vec2
from direct.interval.IntervalGlobal import Sequence, Func

from create_geomnode import SphericalShape, RightTriangularPrism
from utils import create_line_node
from basic_character import Status, Direction, Sensor
from create_maze_3d import Corners


class AirFrame(NodePath):

    def __init__(self, body_color, mask):
        super().__init__(BulletRigidBodyNode(body_color.name.lower()))
        self.set_collide_mask(BitMask32.bit(5))
        self.node().set_kinematic(True)
        self.node().set_ccd_motion_threshold(1e-7)
        self.node().set_ccd_swept_sphere_radius(0.5)

        self.create_body(body_color.value)
        self.create_wings()

    def create_body(self, body_color):
        body = SphericalShape(radius=0.5)
        body.set_scale(Vec3(1))  # if oval, set Vec3(1, 1.5, 1).
        body.set_color(body_color)
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


class Aircraft:

    def __init__(self, world, maze_builder, body_color, bit,
                 linear_velocity=5, angular_velocity=100):
        self.world = world
        self.maze = maze_builder
        self.mask = BitMask32.bit(bit)

        self.root_np = NodePath('root')
        self.direction_np = NodePath('direction')
        self.body = AirFrame(body_color, self.mask)
        self.body.reparent_to(self.direction_np)
        self.direction_np.reparent_to(self.root_np)
        self.root_np.reparent_to(base.render)
        self.world.attach(self.body.node())

        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.vertical_velocity = 3

        self.sensors = [sensor for sensor in self.create_sensors()]
        self.initialize()
        # self.draw_debug_lines()

        self.body_color = body_color

    def initialize(self):
        self.total_distance = 0
        self.total_angle = 0
        self.total_ascent = 0

        self.dead_end = False
        self.stop = False
        self.state = None

    def set_up(self, corner):
        self.start_z = self.maze.wall_size.z - 0.5 + self.maze.get_maze_pos().z

        match corner:

            case Corners.TOP_RIGHT:
                start_xy = self.maze.top_right - self.maze.wall_size.xy
                hpr = Vec3(180, 0, 0)

            case Corners.BOTTOM_LEFT:
                start_xy = self.maze.bottom_left + self.maze.wall_size.xy
                hpr = Vec3(0, 0, 0)

            case Corners.TOP_LEFT:
                diff = Vec2(self.maze.wall_size.x, 0)
                start_xy = self.maze.top_left + diff
                hpr = Vec3(180, 0, 0)

            case Corners.BOTTOM_RIGHT:
                diff = Vec2(self.maze.wall_size.x, 0)
                start_xy = self.maze.bottom_right - diff
                hpr = Vec3(0, 0, 0)

        pos = Point3(start_xy, self.start_z + 1)
        self.root_np.set_pos(pos)
        self.direction_np.set_hpr(hpr)

    def get_backward_pos(self, distance):
        """Return backward position.
           Args:
               distance(float): must be positive.
        """
        backward_vector = self.direction_np.get_quat(base.render).get_forward() * -1
        return self.root_np.get_pos() + backward_vector * distance

    def create_sensors(self):
        for direction in Direction.around():
            if direction != Direction.BACKWARD:
                sensor = Sensor(self.world, direction)
                sensor.reparent_to(self.direction_np)
                yield sensor

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
        pos_from = self.root_np.get_pos()
        mask = BitMask32.bit(2) | self.mask

        for sensor in self.sensors:
            if not sensor.detect_obstacles(pos_from, mask=mask):
                yield sensor.direction

    def turn(self, direction, dt, max_angle=90):
        rotate_direction = direction.get_direction()
        angle = self.angular_velocity * dt

        if (total := self.total_angle + angle) >= max_angle:
            diff = max_angle - self.total_angle
            self.direction_np.set_h(self.direction_np.get_h() - diff * rotate_direction)
            self.total_angle = 0
            return True

        self.total_angle = total
        self.direction_np.set_h(self.direction_np.get_h() - angle * rotate_direction)

    def move_forward(self, dt, max_distance=2):
        forward_vector = self.direction_np.get_quat(base.render).get_forward()
        distance = self.linear_velocity * dt
        self.total_distance += distance

        if (diff := self.total_distance - max_distance) >= 0:
            self.root_np.set_pos(self.root_np.get_pos() + forward_vector * (distance - diff))
            self.total_distance = 0
            return True

        self.root_np.set_pos(self.root_np.get_pos() + forward_vector * distance)

    def check_downward(self, n=5):
        current_pos = self.root_np.get_pos()
        below_pos = current_pos - Vec3(0, 0, 2)

        ts_from = TransformState.make_pos(below_pos)
        ts_to = TransformState.make_pos(current_pos)
        shape = BulletSphereShape(0.5)

        if (result := self.world.sweep_test_closest(
                shape, ts_from, ts_to, BitMask32.bit(n), 0.0)).has_hit():
            if result.get_node() != self.body.node():
                return True

    def lift(self, dt, max_ascent=1, direction=1):
        distance = self.vertical_velocity * 2 * dt

        if (total := self.total_ascent + distance) >= max_ascent:
            z = -8.5 + max_ascent if direction > 0 else -8.5
            self.root_np.set_z(z)
            self.total_ascent = 0
            return True

        self.root_np.set_z(self.root_np.get_z() + distance * direction)
        self.total_ascent = total

    def detect_collision(self, other):
        if (result := self.world.contact_test_pair(
                self.body.node(), other.node())).get_num_contacts() > 0:
            return result

    def get_relative_pos(self, pos):
        return self.root_np.get_relative_point(self.direction_np, pos)

    def close_route(self):
        backward_pos = self.get_backward_pos(self.maze.wall_size.y)
        z = self.start_z - self.maze.get_maze_pos().z
        pos = Point3(backward_pos.xy, z)
        scale = Vec3(self.maze.wall_size.xy, 1)

        _ = self.maze.make_block('closed', pos, scale, self.mask, True)
        # block.set_color(self.body_color.value)
        # block.show()

    def change_to_movement(self, direction):
        match direction:

            case Direction.FORWARD:
                return Status.MOVE

            case Direction.LEFTWARD:
                return Status.LEFT_TURN

            case Direction.RIGHTWARD:
                return Status.RIGHT_TURN

            case Direction.BACKWARD:
                return Status.U_TURN

    def change_direction(self):
        if not (directions := [d for d in self.detect_route()]):
            self.dead_end = True
            return self.change_to_movement(Direction.BACKWARD)

        if len(directions) >= 2:
            if self.dead_end:
                self.close_route()
                self.dead_end = False
            random.shuffle(directions)

        return self.change_to_movement(directions[0])

    def get_next_movement(self):
        if self.maze.is_outside(self.root_np.get_pos().xy):
            base.messenger.send('finish')
            return Status.FINISH

        elif self.root_np.get_z() > self.start_z:
            return Status.CHECK_DOWNWARD

        return Status.STOP

    def update(self, dt):
        if not self.stop:
            match self.state:

                case Status.STOP:
                    self.state = self.change_direction()

                case Status.MOVE:
                    if self.move_forward(dt):
                        self.state = self.get_next_movement()

                case Status.LEFT_TURN:
                    # if self.turn(Direction.LEFTWARD.get_direction(), dt):
                    if self.turn(Direction.LEFTWARD, dt):
                        self.state = Status.MOVE

                case Status.RIGHT_TURN:
                    # if self.turn(Direction.RIGHTWARD.get_direction(), dt):
                    if self.turn(Direction.RIGHTWARD, dt):
                        self.state = Status.MOVE

                case Status.U_TURN:
                    # if self.turn(Direction.LEFTWARD.get_direction(), dt, 180):
                    if self.turn(Direction.LEFTWARD, dt, 180):
                        self.state = Status.MOVE

                case Status.LIFT_UP:
                    if self.lift(dt):
                        self.state = Status.MOVE

                case Status.LIFT_DOWN:
                    if self.lift(dt, direction=-1):
                        self.state = Status.STOP

                case Status.CHECK_DOWNWARD:
                    if not self.check_downward():
                        self.state = Status.LIFT_DOWN

    def start(self, duration):
        xy = self.root_np.get_pos().xy
        pos = Point3(xy, self.start_z)

        def _start():
            self.state = Status.STOP

        Sequence(
            self.root_np.posInterval(duration, pos),
            Func(_start)
        ).start()