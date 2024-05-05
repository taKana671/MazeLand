from enum import Enum, auto
from collections import deque

from direct.actor.Actor import Actor
from panda3d.bullet import BulletSphereShape, BulletCapsuleShape, ZUp
from panda3d.bullet import BulletRigidBodyNode
from panda3d.core import PandaNode, NodePath, TransformState
from panda3d.core import Vec3, Point3, BitMask32

from create_maze_3d import Space


class Motions:

    FORWARD = auto()
    BACKWARD = auto()
    LEFT = auto()
    RIGHT = auto()
    TURN = auto()


class MazeActor(NodePath):

    RUN = 'run'
    WALK = 'walk'

    def __init__(self):
        super().__init__(BulletRigidBodyNode('maze_actor'))
        h, w = 6, 1.2
        shape = BulletCapsuleShape(w, h - 2 * w, ZUp)
        self.node().add_shape(shape)
        self.node().set_kinematic(True)
        self.node().set_ccd_motion_threshold(1e-7)
        self.node().set_ccd_swept_sphere_radius(0.5)
        self.set_collide_mask(BitMask32.bit(1) | BitMask32.bit(4))
        self.set_scale(0.4)  # 0.5
        # self.set_scale(0.5)  # 0.5

        self.actor = Actor(
            'models/ralph/ralph.egg',
            {self.RUN: 'models/ralph/ralph-run.egg',
             self.WALK: 'models/ralph/ralph-walk.egg'}
        )
        self.actor.set_transform(TransformState.make_pos(Vec3(0, 0, -2.5)))
        # self.actor.set_transform(TransformState.make_pos(Vec3(0, 0, -3)))
        self.actor.set_name('ralph')
        self.actor.reparent_to(self)

    def play_anim(self, motion):
        match motion:
            case Motions.FORWARD:
                anim = self.WALK
                rate = 1
            case Motions.BACKWARD:
                anim = self.WALK
                rate = -1
            case Motions.TURN:
                anim = self.WALK
                rate = 1
            case _:
                if self.actor.get_current_anim() is not None:
                    self.actor.stop()
                    self.actor.pose(self.WALK, 5)
                return

        if self.actor.get_current_anim() != anim:
            self.actor.loop(anim)
            self.actor.set_play_rate(rate, anim)


class MazeWalker:

    def __init__(self, world):
        self.world = world
        self.root_np = NodePath('root')
        self.direction_np = NodePath('direction')
        self.actor = MazeActor()
        self.actor.reparent_to(self.direction_np)
        self.direction_np.reparent_to(self.root_np)
        self.root_np.reparent_to(base.render)
        self.world.attach(self.actor.node())

        self.linear_velocity = 5
        self.angular_velocity = 100

    def set_pos(self, pos):
        self.root_np.set_pos(pos)

    def get_pos(self):
        return self.root_np.get_pos()

    def turn(self, direction, dt):
        if not direction:
            return

        angle = direction * self.angular_velocity * dt
        self.direction_np.set_h(self.direction_np.get_h() + angle)

    def move(self, direction, dt):
        if not direction:
            return

        current_pos = self.get_pos()
        forward_vector = self.direction_np.get_quat(base.render).get_forward()
        speed = self.linear_velocity if direction < 0 else self.linear_velocity / 2
        pos_to = current_pos + forward_vector * direction * speed * dt

        # offset = self.detect_collision()
        # print(offset)

        if ground_pos := self.cast_ray_downward(pos_to):
            next_pos = ground_pos + Vec3(0, 0, 1.5)  #  + offset

            if self.predict_collision(current_pos, next_pos):
                # if offset != 0:
                #     self.set_pos(current_pos + offset)
                return

            self.set_pos(next_pos)

    def play_anim(self, motion):
        self.actor.play_anim(motion)

    def cast_ray_downward(self, pos_from):
        pos_to = pos_from + Vec3(0, 0, -30)

        if (result := self.world.ray_test_closest(
                pos_from, pos_to, mask=BitMask32.bit(1))).has_hit():
            return result.get_hit_pos()

        return None

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
        shape = BulletSphereShape(0.6)

        if (result := self.world.sweep_test_closest(
                shape, ts_from, ts_to, BitMask32.bit(2), 0.0)).has_hit():
            return result

        return None


class MazeWalkerController:

    def __init__(self, world, maze_builder):
        self.world = world
        self.maze_builder = maze_builder

        self.walker = MazeWalker(self.world)
        xy = self.maze_builder.get_exit()
        xy.y += self.maze_builder.wall_wd.y * 2
        print('walker start pos', Point3(xy, -7))
        self.walker.set_pos(Point3(xy, -7))

        self.current_space = None
        self.queue = deque()

    def update(self, motions, dt):
        # print(self.walker.get_pos())
        move_direction = 0
        rotate_direction = 0
        motion = None

        if Motions.LEFT in motions:
            rotate_direction += 1
            motion = Motions.TURN

        if Motions.RIGHT in motions:
            rotate_direction -= 1
            motion = Motions.TURN

        if Motions.FORWARD in motions:
            move_direction += -1
            motion = Motions.FORWARD

        if Motions.BACKWARD in motions:
            move_direction += 1
            motion = Motions.BACKWARD

        self.walker.turn(rotate_direction, dt)
        self.walker.move(move_direction, dt)
        self.walker.play_anim(motion)

        pos = self.walker.root_np.get_pos()
        self.current_space = Space(int(pos.y), int(pos.x))
        self.move_direction = move_direction

    def is_walker_in_maze(self):
        pos = self.walker.get_pos()
        return self.maze_builder.is_in_maze(pos)

    def navigate(self, pos):
        return self.walker.root_np.get_relative_point(self.walker.direction_np, pos)

    def get_walker_pos(self):
        return self.walker.root_np.get_pos()

    def check_current_pos(self):
        if hit := self.walker.cast_ray_downward_brick(self.get_walker_pos()):
            # print(hit.get_node().get_name(), hit.get_hit_pos())
            name = hit.get_node().get_name()
            _, r, c = name.split('_')
            space = Space(int(r), int(c))

            if len(self.queue) == 0:
                self.queue.append(space)
                return None


            if self.queue[-1] != space:
                if len(self.queue) >= 1:
                    # 後戻り
                    if self.queue[-1] == space:
                        self.queue.pop()

                        if len(self.queue) >= 2:
                            return self.queue[-2]
                    else:
                        self.queue.append(space)
                        if len(self.queue) >= 2:
                            return self.queue[-2]
                # else:
                #     self.queue.append(space)

                self.queue.append(space)

            
            # if self.queue[-1] != space:
            #     if len(self.queue) >= 2:
            #         # 後戻り
            #         if self.queue[-2] == space:
            #             self.queue.pop()

            #             if len(self.queue) >= 3:
            #                 return self.queue[-3]
            #         else:
            #             self.queue.append(space)
            #             if len(self.queue) >= 3:
            #                 return self.queue[-3]
            #     # else:
            #     #     self.queue.append(space)

            #     self.queue.append(space)
          
        return None
                


