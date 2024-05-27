import math
import random
from collections import deque
from typing import NamedTuple

import numpy as np
from panda3d.bullet import BulletSphereShape, BulletCapsuleShape, ZUp
from panda3d.bullet import BulletRigidBodyNode
from panda3d.core import PandaNode, NodePath, TransformState
from panda3d.core import Vec3, Point3, BitMask32, Point2, Quat, LColor

from direct.interval.IntervalGlobal import ProjectileInterval, Parallel, Sequence, Func

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
        self.set_collide_mask(BitMask32.bit(4))


class PassingPoints(NamedTuple):

    start: Point3
    mid: Point3
    end: Point3


class MazeWalker:

    def __init__(self, world, moving_distance):
        self.world = world

        self.root_np = NodePath('root')
        self.direction_np = NodePath('direction')

        self.character = Character()
        self.character.reparent_to(self.direction_np)
        self.direction_np.reparent_to(self.root_np)
        self.root_np.reparent_to(base.render)
        self.world.attach(self.character.node())

        self.moving_distance = moving_distance
        self.angular_velocity = 200
        self.max_acceleration = 15
        self.character_z = 0.5

        self.total = 0
        self.acceleration = 0

        self.sensors = [sensor for sensor in self.create_sensors()]

    def create_sensors(self):
        for direction in Direction.around():
            sensor = Sensor(self.world, direction, -1)
            sensor.reparent_to(self.direction_np)
            yield sensor

    def set_pos(self, pos):
        self.root_np.set_pos(pos)

    def get_pos(self):
        return self.root_np.get_pos()

    def get_passing_points(self, direction):
        start_pt = self.get_pos()
        forward_vector = self.direction_np.get_quat(base.render).get_forward() * direction
        to_pos = forward_vector * self.moving_distance + start_pt
        hit = self.cast_ray_downward(to_pos)
        end_pt = hit.get_hit_pos() + Vec3(0, 0, self.character_z)

        mid_pt = (start_pt + end_pt) / 2
        mid_pt.z += 1

        self.passing_pts = PassingPoints(start_pt, mid_pt, end_pt)

    def bernstein(self, n, k):
        coef = math.factorial(n) / (math.factorial(k) * math.factorial(n - k))
        return coef * self.total ** k * (1 - self.total) ** (n - k)

    def bezier_curve(self, dt):
        self.total += dt

        if self.total > 1:
            self.total = 1

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
                if not sensor.detect_obstacles(pos_from, bit=4):
                    return True

    def cast_ray_downward(self, pos, from_delta=3, to_delta=-10, mask=BitMask32.bit(1)):
        pos_from = pos + Vec3(0, 0, from_delta)
        pos_to = pos + Vec3(0, 0, to_delta)
        # print('cast_ray_downward', pos_from, pos_to)
        if (result := self.world.ray_test_closest(
                pos_from, pos_to, mask=mask)).has_hit():
            return result

        return None

    def turn(self, rotate_direction, dt, max_angle=90):
        angle = self.angular_velocity * dt

        if (total := self.total + angle) >= max_angle:
            diff = max_angle - self.total
            self.direction_np.set_h(self.direction_np.get_h() + diff * rotate_direction)
            self.total = 0
            return True

        self.total = total
        self.direction_np.set_h(self.direction_np.get_h() + angle * rotate_direction)

    def move(self, dt):
        next_pt = self.bezier_curve(dt)
        self.set_pos(next_pt)

        if self.total == 1:
            self.total = 0
            self.set_pos(self.passing_pts.end)
            return True

    def jump(self, dt):
        if self.acceleration <= -1 * self.max_acceleration:
            hit = self.cast_ray_downward(self.root_np.get_pos())
            landing_pos = hit.get_hit_pos() + Vec3(0, 0, self.character_z)
            self.root_np.set_z(landing_pos.z)
            return True

        next_z = self.acceleration * dt
        self.root_np.set_z(self.root_np.get_z() + next_z)
        self.acceleration -= 0.98 * 0.25

    def start_projectile(self, aircraft_pos):
        current_pos = self.root_np.get_pos()
        result = self.cast_ray_downward(current_pos)
        end_z = result.get_hit_pos().z
        way_pt = current_pos + Vec3(0, 0, 3)

        ProjectileInterval.gravity = 9.81 / 4

        self.projectile_seq = Parallel(
            self.direction_np.hprInterval(3, Vec3(360, 270, 270)),
            ProjectileInterval(
                self.root_np,
                startPos=current_pos,
                wayPoint=way_pt,
                timeToWayPoint=1,
                endZ=end_z,
                gravityMult=0.5
            )
        )
        self.projectile_seq.start()


class MazeWalkerController:

    def __init__(self, world, maze_builder, walker_q):
        self.world = world
        self.maze = maze_builder
        self.walker = MazeWalker(self.world, self.maze.wall_size.x)
        self.trace_q = walker_q

        self.orient = -1
        xy = self.maze.get_exit()
        # print('walker start pos', Point3(xy, -5))
        self.walker.set_pos(Point3(xy, -8.5))
        # self.walker.set_pos(Point3(18, -18, -9))
        self.state = None

    @property
    def walker_pos(self):
        return self.walker.root_np.get_pos()

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

            case _:
                return None

    def update(self, direction, dt):
        match self.state:
            case Status.STOP:
                if status := self.change_direction(direction):
                    self.state = status

            case Status.LEFT_TURN:
                if self.walker.turn(Direction.LEFTWARD.get_direction(self.orient), dt):
                    self.state = Status.STOP

            case Status.RIGHT_TURN:
                if self.walker.turn(Direction.RIGHTWARD.get_direction(self.orient), dt):
                    self.state = Status.STOP

            case Status.MOVE:
                if self.walker.move(dt):
                    if self.maze.is_outside(self.walker_pos.xy):
                        print('finish')
                        self.state = Status.FINISH
                    else:
                        self.state = Status.STOP

            case Status.DO_JUMP:
                if self.walker.jump(dt):
                    self.state = Status.STOP

            case Status.CRASH:
                if not self.walker.projectile_seq.is_playing():
                    self.state = Status.FINISH

        return self.walker.root_np.get_pos()

    def navigate(self, pos):
        return self.walker.root_np.get_relative_point(self.walker.direction_np, pos)

    def crash(self, aircraft_pos):
        if self.state != Status.CRASH:
            self.state = Status.CRASH
            self.walker.start_projectile(aircraft_pos)

    def start(self):
        self.state = Status.STOP