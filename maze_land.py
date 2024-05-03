import sys
from datetime import datetime

from panda3d.bullet import BulletWorld, BulletSphereShape
from panda3d.bullet import BulletDebugNode, BulletRigidBodyNode
from panda3d.core import Vec3, NodePath, BitMask32, Point3, Quat
from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
from direct.showbase.InputStateGlobal import inputState
from panda3d.core import load_prc_file_data

import numpy as np

from create_maze_3d import MazeBuilder
from lights import BasicAmbientLight, BasicDayLight
from scene import Scene
from aircraft import AircraftController
from maze_walker import MazeWalkerController, Motions


load_prc_file_data("", """
    textures-power-2 none
    gl-coordinate-system default
    window-title Panda3D Avoid Balls
    filled-wireframe-apply-shader true
    stm-max-views 8
    stm-max-chunk-count 2048""")


class Maze3D(ShowBase):

    def __init__(self):
        super().__init__()
        self.disable_mouse()
        self.world = BulletWorld()
        self.world.set_gravity(Vec3(0, 0, -9.81))

        self.debug = self.render.attach_new_node(BulletDebugNode('debug'))
        self.world.set_debug_node(self.debug.node())

        self.maze_builder = MazeBuilder(self.world, 21, 21)
        self.maze_builder.build()


        self.walker_controller = MazeWalkerController(self.world, self.maze_builder)
        self.floater = NodePath('floater')
        self.floater.set_z(1)   # 3
        self.floater.reparent_to(self.walker_controller.walker.root_np)
        pos = self.walker_controller.navigate(Vec3(0, 4, 2))   # Vec3(0, 10, 2) -> Vec3(0, 4, 3)
        self.camera.reparent_to(self.walker_controller.walker.root_np)
        self.camera.set_pos(pos)
        self.camera.look_at(self.floater)
        self.camLens.set_fov(90)

        # ############### aircraft part ############################
        # self.aircraft_controller = AircraftController(self.world, self.maze_builder)

        # self.floater = NodePath('floater')
        # self.floater.set_z(5)
        # self.floater.reparent_to(self.aircraft_controller.aircraft.root_np)

        # self.camera.reparent_to(self.aircraft_controller.aircraft.root_np)
        # self.camera.set_pos(self.aircraft_controller.get_relative_pos(Point3(0, -2, 10)))  # Point3(0, -8, 6)
        # self.camera.look_at(self.floater)
        # self.camLens.set_fov(90)

        # self.camera.set_pos(0, 0, 1000)
        # self.camera.look_at(0, 0, 0)
        # ###########################################

        self.ambient_light = BasicAmbientLight()
        self.ambient_light.reparent_to(self.render)
        self.day_light = BasicDayLight()
        self.day_light.reparent_to(self.render)
        self.scene = Scene(self.world)

        inputState.watch_with_modifiers('forward', 'arrow_up')
        inputState.watch_with_modifiers('backward', 'arrow_down')
        inputState.watch_with_modifiers('left', 'arrow_left')
        inputState.watch_with_modifiers('right', 'arrow_right')

        self.accept('escape', sys.exit)
        self.accept('d', self.toggle_debug)
        self.accept('p', self.print_info)
        self.taskMgr.add(self.update, 'update')

    def toggle_debug(self):
        if self.debug.is_hidden():
            self.debug.show()
        else:
            self.debug.hide()

    def print_info(self):
        walker_pos = self.walker_controller.get_walker_pos()
        print('walker_pos', walker_pos)
        print('camera_pos', self.camera.get_pos(self.render))
        print('backward_pos', self.walker_controller.navigate(Vec3(0, 4, 3)) + walker_pos)
        print('camera reative pos', self.camera.get_pos())

    # def rotate_camera(self, add_angle, max_angle, direction, walker_pos, camera_pos, target_angle=0):

    #     target_angle += add_angle * direction
    #     print('target angle', target_angle)
    #     if abs(target_angle) >= abs(max_angle):
    #         return max_angle

    #     q = Quat()
    #     q.set_from_axis_angle(target_angle, Vec3.up())
    #     r = q.xform(camera_pos - walker_pos)
    #     rotated_pos = walker_pos + r
    #     print('rotated pos', rotated_pos)

    #     if (result := self.world.ray_test_closest(
    #             rotated_pos, walker_pos, mask=BitMask32.bit(1))).has_hit():
    #         if result.get_node().get_name() == 'maze_actor':
    #             return rotated_pos
    #     # self.rotate_camera(add_angle, max_angle, direction, walker_pos, camera_pos, target_angle)


    def rotate_camera(self, angle, point, object_pos):
        object_pos = self.camera.get_pos()
        point = Point3(0, 0, 0)
        q = Quat()
        q.set_from_axis_angle(angle, Vec3.up())
        r = q.xform(object_pos - point)
        rotated_pos = point + r
        return rotated_pos



        # object_pos = self.camera.get_pos(self.render)
        # q = Quat()
        # q.set_from_axis_angle(angle, Vec3.up())
        # r = q.xform(object_pos - point)
        # rotated_pos = point + r
        # return rotated_pos

    def get_rotation_angle(self, pos_a, pos_b, pos_c):
        # pos_a: camera pos, pos_b: walker pos, pos_c: backward pos
        # pos_a: backward pos, pos_b: walker pos, po_c 
        a = np.array(pos_a.xy)
        b = np.array(pos_b.xy)
        c = np.array(pos_c.xy)

        vec_a = a - b
        vec_c = c - b

        vec_a_len = np.linalg.norm(vec_a)
        vec_c_len = np.linalg.norm(vec_c)
        inner_product = np.inner(vec_a, vec_c)
        cos = inner_product / (vec_a_len * vec_c_len)
        rad = np.arccos(cos)
        degree = np.rad2deg(rad)
        cross_product = np.cross(vec_a, vec_c)
        # direction = -1 if cross_product <= 0 else 1
        direction = 1 if cross_product >= 0 else -1
        
        return degree, direction

    def control_camera(self, dt):
        # if self.walker_controller.is_walker_in_maze():
        #     import pdb; pdb.set_trace()       
        # print(walker_pos)

        walker_pos = self.walker_controller.get_walker_pos()
        camera_pos = self.camera.get_pos(self.render)
        # camera_pos = self.camera_controller.get_pos(self.render)

        # print('ray test all')
        # for hit in self.world.ray_test_all(walker_pos, camera_pos, mask=BitMask32.bit(4)).get_hits():
        #     print(hit.get_node().get_name())
        # print('----------------------------')


        # print('backward_pos', self.walker_controller.navigate(Vec3(0, 4, 3)) + walker_pos)
        self.detect_camera_collision()

        if (result := self.world.ray_test_closest(
                camera_pos, walker_pos, mask=BitMask32.bit(4))).has_hit():
            if result.get_node().get_name() != 'maze_actor':
     
                backward_pos = self.walker_controller.navigate(Vec3(0, 4, 3)) + walker_pos
                degree, direction = self.get_rotation_angle(camera_pos, walker_pos, backward_pos)
                next_pos = self.rotate_camera(degree * direction, walker_pos, camera_pos)

                self.camera.set_pos(next_pos)
                self.camera.look_at(self.floater)

        
         
        # grid_space = Point3(int(walker_pos.x), int(walker_pos.y), walker_pos.z)
        # diff = grid_space - walker_pos
        # print(diff)
        # self.camera.set_pos(Vec3(0, 4, 3) + diff)

        # if not self.in_maze:
        #     if -19 < int(walker_pos.x) < 19 and -19 < int(walker_pos.y) < 19:
        #         self.in_maze = True
        #         self.camera.reparent_to(self.camera_root)
        #         self.camera.set_pos(0, 0, 0)
        #         self.camera.look_at(self.floater)
        #         self.before_row = int(walker_pos.y)
        #         self.before_col = int(walker_pos.x)
        #         self.camera.look_at(self.walker_controller.walker.actor)

                
        # ********************************************************
        # print('camera pos', self.camera.get_pos(base.render), 'worker_pos', walker_pos)
        # if self.camera.get_x(base.render) != -18:
        #     diff = -18 - walker_pos.x
        #     self.camera.set_x(diff)
        
        # camera_pos = self.camera.get_pos(base.render)
        # if (result := self.world.ray_test_closest(
        #         camera_pos, walker_pos, BitMask32.bit(1))).has_hit():
        #     print(result.get_node().get_name())
        #     if result.get_node().get_name() != 'maze_actor':

        #         self.camera.set_h(self.camera.get_h() + 90)
        #         # self.camera.look_at(self.floater)
      




    def control_walker(self, dt):
        motions = []

        if inputState.is_set('forward'):
            motions.append(Motions.FORWARD)
        if inputState.is_set('backward'):
            motions.append(Motions.BACKWARD)
        if inputState.is_set('left'):
            motions.append(Motions.LEFT)
        if inputState.is_set('right'):
            motions.append(Motions.RIGHT)

        self.walker_controller.update(motions, dt)

    def update(self, task):
        dt = globalClock.get_dt()
        self.control_walker(dt)
        self.control_camera(dt)

        # self.aircraft_controller.update(dt)
        self.world.do_physics(dt)
        return task.cont


if __name__ == '__main__':
    app = Maze3D()
    app.run()
