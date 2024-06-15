import math
from typing import NamedTuple

import numpy as np
from panda3d.bullet import BulletCapsuleShape, ZUp
from panda3d.bullet import BulletRigidBodyNode
from panda3d.core import NodePath, TransformState
from panda3d.core import Vec2, Vec3, Point3, BitMask32
from direct.interval.IntervalGlobal import ProjectileInterval, Parallel

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

    def __init__(self, world, maze_builder, walker_q, orient=-1):
        self.world = world
        self.maze = maze_builder
        self.trace_q = walker_q
        self.orient = orient

        self.root_np = NodePath('root')
        self.direction_np = NodePath('direction')
        self.body = Character()
        self.body.reparent_to(self.direction_np)
        self.direction_np.reparent_to(self.root_np)
        self.root_np.reparent_to(base.render)
        self.world.attach(self.body.node())

        self.moving_distance = self.maze.wall_size.x
        self.angular_velocity = 200
        self.max_acceleration = 15
        self.body_z = 0.5

        self.sensors = [sensor for sensor in self.create_sensors()]
        self.initialize()

    def initialize(self):
        self.total = 0
        self.acceleration = 0
        self.state = None

    def set_up(self):
        xy = self.maze.top_left + Vec2(self.maze.wall_size.x, 0)
        hit_pos = self.cast_ray_downward(Point3(xy, 0), from_delta=30, to_delta=-30)
        z = hit_pos.z + self.body_z
        self.root_np.set_pos(Point3(xy, z))
        self.direction_np.set_hpr(Vec3(0, 0, 0))

    def create_sensors(self):
        for direction in Direction.around():
            sensor = Sensor(self.world, direction, -1)
            sensor.reparent_to(self.direction_np)
            yield sensor

    def calc_passing_points(self, direction):
        start_pt = self.root_np.get_pos()

        relative_direction = direction.get_direction(self.orient)
        forward_vector = self.direction_np.get_quat(base.render).get_forward() * relative_direction
        to_pos = forward_vector * self.moving_distance + start_pt

        # cannot get outside of the entrance.
        if (hit_pos := self.cast_ray_downward(to_pos)).y > self.maze.top_left.y:
            return False

        end_pt = hit_pos + Vec3(0, 0, self.body_z)
        mid_pt = (start_pt + end_pt) / 2
        mid_pt.z += 1

        self.passing_pts = PassingPoints(start_pt, mid_pt, end_pt)
        return True

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
                if not sensor.detect_obstacles(pos_from, mask=BitMask32.bit(4)):
                    return True

    def cast_ray_downward(self, pos, from_delta=3, to_delta=-10, mask=BitMask32.bit(1)):
        pos_from = pos + Vec3(0, 0, from_delta)
        pos_to = pos + Vec3(0, 0, to_delta)

        if (result := self.world.ray_test_closest(
                pos_from, pos_to, mask=mask)).has_hit():
            return result.get_hit_pos()

        return None

    def turn(self, direction, dt, max_angle=90):
        rotate_direction = direction.get_direction(self.orient)
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
        self.root_np.set_pos(next_pt)

        if self.total == 1:
            self.total = 0
            self.root_np.set_pos(self.passing_pts.end)
            return True

    def jump(self, dt):
        if self.acceleration <= -1 * self.max_acceleration:
            hit_pos = self.cast_ray_downward(self.root_np.get_pos())
            landing_pos = hit_pos + Vec3(0, 0, self.body_z)
            self.root_np.set_z(landing_pos.z)
            return True

        next_z = self.acceleration * dt
        self.root_np.set_z(self.root_np.get_z() + next_z)
        self.acceleration -= 0.98 * 0.25

    def change_to_movement(self, direction):
        match direction:

            case Direction.FORWARD | Direction.BACKWARD:
                if not self.check_route(direction) or \
                        not self.calc_passing_points(direction):
                    return None

                self.trace_q.append(self.passing_pts)
                return Status.MOVE

            case Direction.LEFTWARD:
                return Status.LEFT_TURN

            case Direction.RIGHTWARD:
                return Status.RIGHT_TURN

            case Direction.UPWARD:
                self.acceleration = self.max_acceleration
                return Status.DO_JUMP

            case _:
                return None

    def update(self, direction, dt):
        match self.state:
            case Status.STOP:
                if status := self.change_to_movement(direction):
                    self.state = status

            case Status.LEFT_TURN:
                if self.turn(Direction.LEFTWARD, dt):
                    self.state = Status.STOP

            case Status.RIGHT_TURN:
                if self.turn(Direction.RIGHTWARD, dt):
                    self.state = Status.STOP

            case Status.MOVE:
                if self.move(dt):
                    if self.maze.is_outside(self.root_np.get_pos().xy):
                        self.finish()
                    else:
                        self.state = Status.STOP

            case Status.DO_JUMP:
                if self.jump(dt):
                    self.state = Status.STOP

            case Status.CRASH:
                if not self.projectile_seq.is_playing():
                    self.finish()

        return self.root_np.get_pos()

    def navigate(self, offset):
        pos = offset + self.root_np.get_pos()
        return self.root_np.get_relative_point(self.direction_np, pos)

    def crash(self):
        if self.state != Status.CRASH:
            current_pos = self.root_np.get_pos()
            hit_pos = self.cast_ray_downward(current_pos)
            self.projectile_seq = ProjectileSequence(
                self.root_np, self.direction_np, current_pos, hit_pos.z)
            self.projectile_seq.start()
            self.state = Status.CRASH

    def finish(self):
        self.state = Status.FINISH
        base.messenger.send('finish')


class ProjectileSequence(Parallel):

    def __init__(self, root_np, direction_np, current_pos, end_z):
        super().__init__()
        self.set_rotation(direction_np)
        self.set_projectile(root_np, current_pos, end_z)

    def set_rotation(self, direction_np):
        self.append(direction_np.hprInterval(3, Vec3(360, 270, 270)))

    def set_projectile(self, root_np, current_pos, end_z):
        way_pt = current_pos + Vec3(0, 0, 3)
        ProjectileInterval.gravity = 9.81 / 4

        self.append(
            ProjectileInterval(
                root_np,
                startPos=current_pos,
                wayPoint=way_pt,
                timeToWayPoint=1,
                endZ=end_z,
                gravityMult=0.5
            )
        )