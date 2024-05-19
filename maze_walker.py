import math
from collections import deque
from typing import NamedTuple

import numpy as np
from panda3d.bullet import BulletSphereShape, BulletCapsuleShape, ZUp
from panda3d.bullet import BulletRigidBodyNode
from panda3d.core import PandaNode, NodePath, TransformState
from panda3d.core import Vec3, Point3, BitMask32, Point2, Quat, LColor

from create_maze_3d import Space
from basic_character import Sensor, Direction, Status


class Character(NodePath):

    def __init__(self):
        super().__init__(BulletRigidBodyNode('character'))

        height, radius = 7.0, 1.5
        shape = BulletCapsuleShape(radius, height - 2 * radius, ZUp)
        self.node().add_shape(shape)
        self.model = base.loader.loadModel('models/snowman/snowman')
        self.model.setTransform(TransformState.makePos(Vec3(0, 0, -3)))
        self.model.reparentTo(self)
        self.set_scale(0.2)

        self.node().set_kinematic(True)
        self.node().set_ccd_motion_threshold(1e-7)
        self.node().set_ccd_swept_sphere_radius(0.5)
        self.set_collide_mask(BitMask32.bit(1) | BitMask32.bit(4))


class PassingPoints(NamedTuple):

    start: Point3
    mid: Point3
    end: Point3


class MazeWalker:

    def __init__(self, world):
        self.world = world

        self.root_np = NodePath('root')
        self.direction_np = NodePath('direction')

        self.character = Character()
        self.character.reparent_to(self.direction_np)
        self.direction_np.reparent_to(self.root_np)
        self.root_np.reparent_to(base.render)
        self.world.attach(self.character.node())

        self.linear_velocity = 5
        self.angular_velocity = 200
        self.max_acceleration = 15

        self.total_angle = 0
        self.total_dt = 0
        self.acceleration = 0

        self.create_sensors()

    def create_sensors(self):
        self.sensors = [
            Sensor(self.world, Direction.FORWARD, -1),
            Sensor(self.world, Direction.BACKWARD, -1),
            Sensor(self.world, Direction.LEFTWARD, -1),
            Sensor(self.world, Direction.RIGHTWARD, -1),
        ]

        for sensor in self.sensors:
            sensor.reparent_to(self.direction_np)

    def set_pos(self, pos):
        self.root_np.set_pos(pos)

    def get_pos(self):
        return self.root_np.get_pos()

    # def get_passing_points(self, direction=1):
    def get_passing_points(self, direction):
        start_pt = self.get_pos()
        forward_vector = self.direction_np.get_quat(base.render).get_forward() * direction  #  * -1   # 出口からスタートするので-1が必要？変数で可変にするstart_direction=-1みたいに
        # print('forward_vector', forward_vector)
        to_pos = forward_vector * 2 + start_pt
        hit = self.cast_ray_downward(to_pos)
        end_pt = hit.get_hit_pos() + Vec3(0, 0, 0.5)

        mid_pt = (start_pt + end_pt) / 2
        mid_pt.z += 1

        self.passing_pts = PassingPoints(start_pt, mid_pt, end_pt)
        self.total_dt = 0

    def bernstein(self, n, k):
        coef = math.factorial(n) / (math.factorial(k) * math.factorial(n - k))
        return coef * self.total_dt ** k * (1 - self.total_dt) ** (n - k)

    def bezier_curve(self, dt):
        self.total_dt += dt

        if self.total_dt > 1:
            self.total_dt = 1

        n = len(self.passing_pts) - 1
        px = py = pz = 0

        for i, pt in enumerate(self.passing_pts):
            b = self.bernstein(n, i)
            px += np.dot(b, pt[0])
            py += np.dot(b, pt[1])
            pz += np.dot(b, pt[2])

        return Point3(px, py, pz)

    def check_route(self, direction):
        pos_from = self.root_np.get_pos()
        for sensor in self.sensors:
            if direction == sensor.direction:
                if not sensor.detect_obstacles(pos_from):
                    return True

    def cast_ray_downward(self, pos_from):
        pos_to = pos_from + Vec3(0, 0, -30)

        if (result := self.world.ray_test_closest(
                pos_from, pos_to, mask=BitMask32.bit(1))).has_hit():
            return result

        return None

    def turn(self, rotate_direction, dt, max_angle=90):
        angle = self.angular_velocity * dt

        if (total := self.total_angle + angle) >= max_angle:
            diff = max_angle - self.total_angle
            self.direction_np.set_h(self.direction_np.get_h() + diff * rotate_direction)
            self.total_angle = 0
            return True

        self.total_angle = total
        self.direction_np.set_h(self.direction_np.get_h() + angle * rotate_direction)

    def move(self, dt):
        next_pt = self.bezier_curve(dt)
        self.set_pos(next_pt)

        if self.total_dt == 1:
            self.set_pos(self.passing_pts.end)
            return True

    def jump(self, dt):
        if self.acceleration <= -1 * self.max_acceleration:
            return True

        next_z = self.acceleration * dt
        self.root_np.set_z(self.root_np.get_z() + next_z)
        self.acceleration -= 0.98 * 0.25

    def cast_ray_downward_brick(self, pos_from):
        pos_to = pos_from + Vec3(0, 0, -30)

        if (result := self.world.ray_test_closest(
                pos_from, pos_to, mask=BitMask32.bit(5))).has_hit():
            return result

        return None

    def detect_collision(self):
        result = self.world.contact_test(self.actor.node(), use_filter=True)
        offset = Vec3()

        for contact in result.getContacts():
            if contact.getNode1().get_name() != 'terrain':
                # print(contact.getNode0().get_name(), contact.getNode1().get_name())
                mf = contact.get_manifold_point()
                dist = mf.get_distance()
                normal = mf.get_normal_world_on_b()
                offset -= normal * dist * 2    #  * 0.5

        return offset

    def predict_collision(self, pos_from, pos_to):
        ts_from = TransformState.make_pos(pos_from)
        ts_to = TransformState.make_pos(pos_to)
        shape = BulletSphereShape(0.5)

        if (result := self.world.sweep_test_closest(
                shape, ts_from, ts_to, BitMask32.bit(2), 0.0)).has_hit():
            return result

        return None


class MazeWalkerController:

    def __init__(self, world, maze_builder):
        self.world = world
        self.maze_builder = maze_builder
        self.walker = MazeWalker(self.world)

        self.orient = -1
        xy = self.maze_builder.get_exit()
        print('walker start pos', Point3(xy, -5))
        self.walker.set_pos(Point3(xy, -8.5))
        self.state = Status.STOP

        self.trace_q = deque()

    def change_direction(self, direction):

        match direction:

            case Direction.FORWARD | Direction.BACKWARD:
                if not self.walker.check_route(direction):
                    return None

                self.walker.get_passing_points(direction.get_direction(self.orient))
                self.trace_q.append(self.walker.passing_pts)
                return Status.MOVE

            case Direction.LEFTWARD:
                return Status.LEFT_TURN

            case Direction.RIGHTWARD:
                return Status.RIGHT_TURN

            case Direction.UPWARD:
                self.walker.acceleration = self.walker.max_acceleration
                return Status.DO_JUMP

    def update(self, direction, dt):
        match self.state:
            case Status.STOP:
                if direction:
                    self.state = self.change_direction(direction)

            case Status.LEFT_TURN:
                if self.walker.turn(Direction.LEFTWARD.get_direction(self.orient), dt):
                    self.state = Status.STOP

            case Status.RIGHT_TURN:
                if self.walker.turn(Direction.RIGHTWARD.get_direction(self.orient), dt):
                    self.state = Status.STOP

            case Status.MOVE:
                if self.walker.move(dt):
                    self.state = Status.STOP

            case Status.DO_JUMP:
                if self.walker.jump(dt):
                    self.state = Status.STOP

            case _:
                self.state = Status.STOP

    def is_walker_in_maze(self):
        pos = self.walker.get_pos()
        return self.maze_builder.is_in_maze(pos)

    def navigate(self, pos):
        return self.walker.root_np.get_relative_point(self.walker.direction_np, pos)

    def get_walker_pos(self):
        return self.walker.root_np.get_pos()