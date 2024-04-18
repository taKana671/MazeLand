from panda3d.bullet import BulletRigidBodyNode, BulletSphereShape
from panda3d.core import Vec3, NodePath, BitMask32, Point3, LColor

from create_geomnode import SphericalShape
from utils import create_line_node


class Sphere(NodePath):

    def __init__(self, world, initial_pos):
        super().__init__(BulletRigidBodyNode('sphere'))
        self.world = world

        self.set_scale(1)
        self.set_pos(initial_pos)
        self.set_collide_mask(BitMask32.bit(1))

        model = SphericalShape(radius=0.5)
        self.model = self.attach_new_node(model.node())
        end, tip = self.model.get_tight_bounds()
        size = tip - end
        self.node().add_shape(BulletSphereShape(size.z / 2))
        self.node().set_kinematic(True)

        self.node().set_ccd_motion_threshold(1e-7)
        self.node().set_ccd_swept_sphere_radius(0.5)

        self.reparent_to(base.render)
        self.world.attach(self.node())

        self.direction_nd = NodePath('direction')
        self.direction_nd.reparent_to(self)

        self.front = NodePath('front')
        self.front.set_pos(Vec3(0, 1, 0))
        self.front.reparent_to(self.direction_nd)
        # self.front.set_pos(0, 0.26, 0)
        # front_line = create_line_node(pos := self.front.get_pos(), pos + Vec3(0, 1, 0), LColor(0, 0, 1, 1))
        front_line = create_line_node(pos := self.direction_nd.get_pos(self), pos + self.front.get_pos(self), LColor(0, 0, 1, 1))
        front_line.reparent_to(self.direction_nd)

        self.left = NodePath('left')
        self.left.reparent_to(self.direction_nd)
        # self.front.set_pos(0, 0.26, 0)
        left_line = create_line_node(pos := self.left.get_pos(), pos + Vec3(-1, 0, 0), LColor(0, 0, 1, 1))
        left_line.reparent_to(self.direction_nd)

        self.right = NodePath('right')
        self.right.reparent_to(self.direction_nd)
        # self.front.set_pos(0, 0.26, 0)
        right_line = create_line_node(pos := self.right.get_pos(), pos + Vec3(1, 0, 0), LColor(0, 0, 1, 1))
        right_line.reparent_to(self.direction_nd)

    def navigate(self):
        """Return a relative point to enable camera to follow the character
           when the camera's view is blocked by an object like wall.
        """
        return self.get_relative_point(self.direction_nd, Vec3(0, -2, 10))

    def update(self, dt):
        pos_from = self.get_pos()
        pos_to = pos_from + Vec3(0, 1, 0)
        if (result := self.world.ray_test_closest(pos_from, pos_to, mask=BitMask32.bit(2))).has_hit():
            print(result.get_node())
        else:
            self.set_pos(self.get_pos() + Vec3(0, dt * 5, 0))

