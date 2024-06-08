from panda3d.core import AmbientLight, DirectionalLight, Spotlight, PointLight
from panda3d.core import NodePath, PandaNode
from panda3d.core import Vec3, Point3


class BasicAmbientLight(NodePath):

    def __init__(self):
        super().__init__(AmbientLight('ambient_light'))
        self.node().set_color((0.6, 0.6, 0.6, 1))
        self.reparent_to(base.render)
        base.render.set_light(self)


class BasicDayLight(NodePath):

    def __init__(self):
        super().__init__(DirectionalLight('directional_light'))
        self.node().get_lens().set_film_size(200, 200)
        self.node().get_lens().set_near_far(10, 200)
        self.node().set_color((1, 1, 1, 1))
        self.set_pos_hpr(Point3(0, 0, 100), Vec3(-30, -45, 0))
        self.node().set_shadow_caster(True, 8192, 8192)
        # self.node().set_shadow_caster(True, 4096, 4096)

        state = self.node().get_initial_state()
        temp = NodePath(PandaNode('temp_np'))
        temp.set_state(state)
        temp.set_depth_offset(-3)
        # temp.set_depth_offset(-2)
        self.node().set_initial_state(temp.get_state())

        base.render.set_light(self)
        base.render.set_shader_auto()
        self.reparent_to(base.render)
        # self.node().show_frustum()


# class SpotLight(NodePath):

#     def __init__(self):
#         super().__init__(Spotlight('spotight'))
#         self.node().set_color((2, 0, 0, 2))
#         self.node().set_attenuation(Vec3(0, 0, 0.1))
#         self.node().set_exponent(20)
#         self.node().get_lens().set_fov(20)
#         self.node().get_lens().set_near_far(1, 10)
#         # self.reparent_to(base.render)
#         base.render.set_light(self)

#         self.node().set_shadow_caster(True)
#         # self.set_hpr(Vec3(0, 0, 0))
#         # self.node().show_frustum()


# # ************************************
#         self.light = SpotLight()
#         light = Spotlight('spotight')

#         # light.reparent_to(self.direction_nd)
#         np = self.direction_nd.attach_new_node(Spotlight('spotight'))
#         # np.set_pos(Vec3(0, 0.5, 0))
#         # np = self.attachNewNode(Spotlight('spotight'))

#         np.node().set_color((0, 1, 0, 1))
#         np.node().set_attenuation(Vec3(1, 0, 0))
#         np.node().set_exponent(0)
#         np.node().get_lens().set_fov(10)
#         np.node().get_lens().set_near_far(1, 20)
#         # self.reparent_to(base.render)
#         base.render.set_light(np)

#         # np.node().set_shadow_caster(True)
#         # np.node().show_frustum()



#         # self.light.set_pos_hpr(self, Point3(0, 0.5, 0), Vec3(0, 0, 0))
#         # # self.light.set_pos(Point3(0, 0.5, 0))
#         # # self.light.reparent_to(self)
#         # np = self.attach_new_node(self.light)

#         # ************************************