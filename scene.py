from panda3d.bullet import BulletRigidBodyNode, BulletPlaneShape

from panda3d.core import NodePath, PandaNode, CardMaker, Point3
from panda3d.core import BitMask32, Vec3


class Ground(NodePath):

    def __init__(self):
        super().__init__(BulletRigidBodyNode('ground'))
        model = self.create_ground()
        model.reparent_to(self)
        self.set_collide_mask(BitMask32.bit(1))
        self.node().add_shape(BulletPlaneShape(Vec3.up(), 0))

    def create_ground(self):
        model = NodePath(PandaNode('ground_model'))
        card = CardMaker('card')
        size = 2
        half = size / 2
        card.set_frame(-half, half, -half, half)
        max_card = 50

        for y in range(max_card):
            for x in range(max_card):
                g = model.attach_new_node(card.generate())
                g.set_p(-90)
                g.set_pos(Point3(x - 25, y - 25, 0))

        tex = base.loader.load_texture('textures/grass.png')
        model.set_texture(tex)
        model.flatten_strong()
        model.set_pos(0, 0, 0)
        return model

