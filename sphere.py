import random
from enum import Enum, auto

from panda3d.bullet import BulletRigidBodyNode, BulletSphereShape
from panda3d.core import Vec3, NodePath, BitMask32, Point3, LColor, Point2
from direct.interval.IntervalGlobal import Sequence, Func

from create_geomnode import SphericalShape
from utils import create_line_node


class Status(Enum):

    move = auto()
    stop = auto()
    turn = auto()


class Drone(NodePath):

    def __init__(self):
        super().__init__(BulletRigidBodyNode('drone'))
        self.set_scale(1)
        self.set_collide_mask(BitMask32.bit(1))

        model = SphericalShape(radius=0.5)
        self.model = self.attach_new_node(model.node())

        end, tip = self.model.get_tight_bounds()
        size = tip - end
        self.node().add_shape(BulletSphereShape(size.z / 2))
        self.node().set_kinematic(True)
        self.node().set_ccd_motion_threshold(1e-7)
        self.node().set_ccd_swept_sphere_radius(0.5)

        self.direction_nd = NodePath('direction')
        self.direction_nd.reparent_to(self)


class DroneController(NodePath):

    def __init__(self, world, maze_builder):
        super().__init__(BulletRigidBodyNode('sphere'))
        self.world = world
        self.maze_builder = maze_builder

        # self.set_scale(1)

        self.total_distance = 0
        self.dead_end = False

        self.drone = Drone()

        xy = self.maze_builder.get_entrance()
        z = self.maze_builder.wall_size.z - 0.5
        self.drone.set_pos(Point3(xy, z))
        # self.set_collide_mask(BitMask32.bit(1))

        # model = SphericalShape(radius=0.5)
        # self.model = self.attach_new_node(model.node())
        # end, tip = self.model.get_tight_bounds()
        # size = tip - end
        # self.node().add_shape(BulletSphereShape(size.z / 2))
        # self.node().set_kinematic(True)

        # self.node().set_ccd_motion_threshold(1e-7)
        # self.node().set_ccd_swept_sphere_radius(0.5)

        self.drone.reparent_to(base.render)
        self.world.attach(self.drone.node())

        # self.direction_nd = NodePath('direction')
        # self.direction_nd.reparent_to(self)


        self.front = NodePath('front')
        self.front.set_pos(Vec3(0, 1.5, 0))
        self.front.reparent_to(self.direction_nd)
        # self.front.set_pos(0, 0.26, 0)
        # front_line = create_line_node(pos := self.front.get_pos(), pos + Vec3(0, 1, 0), LColor(0, 0, 1, 1))
        front_line = create_line_node(pos := self.direction_nd.get_pos(self), pos + self.front.get_pos(self), LColor(0, 0, 1, 1))
        front_line.reparent_to(self.direction_nd)

        self.left = NodePath('left')
        self.left.set_pos(Vec3(-1.5, 0, 0))
        self.left.reparent_to(self.direction_nd)

        # self.front.set_pos(0, 0.26, 0)
        left_line = create_line_node(pos := self.direction_nd.get_pos(self), pos + self.left.get_pos(self), LColor(0, 0, 1, 1))
        left_line.reparent_to(self.direction_nd)

        self.right = NodePath('right')
        self.right.set_pos(Vec3(1.5, 0, 0))
        self.right.reparent_to(self.direction_nd)
        # self.front.set_pos(0, 0.26, 0)
        right_line = create_line_node(pos := self.direction_nd.get_pos(self), pos + self.right.get_pos(self), LColor(0, 0, 1, 1))
        right_line.reparent_to(self.direction_nd)

        self.state = Status.stop

    def navigate(self):
        """Return a relative point to enable camera to follow the character
           when the camera's view is blocked by an object like wall.
        """
        return self.get_relative_point(self.direction_nd, Vec3(0, -2, 10))

    def find_aisles(self):
        pos_from = self.get_pos()

        for val, sensor in [[0, self.front], [-1, self.left], [1, self.right]]:
            pos_to = sensor.get_pos(base.render)

            # if (result := self.world.ray_test_closest(
            #         pos_from, pos_to, mask=BitMask32.bit(2))).has_hit():
                # print(result.get_node().get_name())

            if not self.world.ray_test_closest(
                    pos_from, pos_to, mask=BitMask32.bit(2)).has_hit():
                yield val
        # print('---------------------------------')

        # for val, sensor in [[0, self.front], [-1, self.left], [1, self.right]]:
        #     pos_to = sensor.get_pos(base.render)
        #     if (result := self.world.ray_test_closest(
        #             pos_from, pos_to, mask=BitMask32.bit(2))).has_hit():
        #         diff = result.get_hit_pos() - pos_from
        #         yield diff.xy

    def remember(self):
        pos_from = self.get_pos()

        for sensor in [self.left, self.right]:
            pos_to = sensor.get_pos(base.render)
            if (result := self.world.ray_test_closest(
                    pos_from, pos_to, mask=BitMask32.bit(2))).has_hit():
                yield result.get_node().get_name()

    def change_state(self):
        self.state = Status.move

    def update(self, dt):
        # forward_vector = self.direction_nd.get_quat(base.render).get_forward()
        # next_pos = current_pos + forward_vector * direction * speed * dt

        match self.state:

            case Status.stop:
                # print(self.get_pos())
                if directions := [d for d in self.find_aisles()]:
                    # import pdb; pdb.set_trace()
                    # print(directions)
                    if len(directions) >= 2:
                        random.shuffle(directions)

                    self.direction = directions[0]
                    # print('selected_directions', self.direction)

                    if self.dead_end:
                        if len(directions) == 1:
                            self.before_pos = self.get_pos()
                        else:
                            scale = Vec3(self.maze_builder.wall_size.xy, 1)
                            mask = BitMask32.bit(1) | BitMask32.bit(2)
                            self.maze_builder.make_wall('closed', self.before_pos, scale, mask, True)
                            self.dead_end = False

                    if self.direction == 0:
                        self.state = Status.move
                    if self.direction == -1:
                        self.state = Status.turn
                        Sequence(
                            self.direction_nd.hprInterval(0.5, (self.direction_nd.get_h() + 90, 0, 0)),
                            Func(self.change_state)
                        ).start()
                    if self.direction == 1:
                        self.state = Status.turn
                        Sequence(
                            self.direction_nd.hprInterval(0.5, (self.direction_nd.get_h() - 90, 0, 0)),
                            Func(self.change_state)
                        ).start()
                else:
                    self.state = Status.turn
                    # self.before_pos = self.get_pos()
                    self.dead_end = True
                    Sequence(
                        self.direction_nd.hprInterval(0.5, (self.direction_nd.get_h() + 180, 0, 0)),
                        Func(self.change_state)
                    ).start()



            case Status.move:
                forward_vector = self.direction_nd.get_quat(base.render).get_forward()
                distance = 1 * 2 * dt

                if self.total_distance + distance > 2:
                    distance = 2 - self.total_distance
                    self.total_distance = 0
                    self.state = Status.stop
                else:
                    self.total_distance += distance

                self.set_pos(self.get_pos() + forward_vector * distance)