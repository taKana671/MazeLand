import sys
from collections import deque
from datetime import datetime

from panda3d.bullet import BulletWorld, BulletSphereShape
from panda3d.bullet import BulletDebugNode, BulletRigidBodyNode
from panda3d.core import Vec3, NodePath, BitMask32, Point3, Quat, LColor, Vec2
from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
from direct.showbase.InputStateGlobal import inputState
from panda3d.core import load_prc_file_data

import numpy as np

from create_maze_3d import MazeBuilder, Space
from lights import BasicAmbientLight, BasicDayLight
from scene import Scene
from aircraft import AircraftController
from maze_walker import MazeWalkerController, Motions
from create_geomnode import SphericalShape


load_prc_file_data("", """
    textures-power-2 none
    gl-coordinate-system default
    window-title Panda3D Avoid Balls
    filled-wireframe-apply-shader true
    stm-max-views 8
    stm-max-chunk-count 2048""")



class CameraController(NodePath):

    def __init__(self):
        super().__init__(BulletRigidBodyNode('camera_controller'))
        self.set_collide_mask(BitMask32.bit(4))
        self.node().set_kinematic(True)
        shape = BulletSphereShape(1.2)
        self.node().add_shape(shape)


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
        # pos = self.walker_controller.navigate(Vec3(0, 4, 2))   # Vec3(0, 10, 2) -> Vec3(0, 4, 3)
        # pos = self.walker_controller.get_walker_pos() + Vec3(0, 4, 2)

        ##################use this if you want camera collosion#################
        self.camera_controller = CameraController()
        # self.camera_controller.reparent_to(self.walker_controller.walker.root_np)
        self.camera_controller.reparent_to(self.render)
        self.world.attach(self.camera_controller.node())

        self.camera.reparent_to(self.camera_controller)
        self.diff = Vec3(0, 4, 3)
        self.camera.look_at(self.floater)
        # self.camera_controller.set_pos(pos)
        self.camLens.set_fov(90)
        #######################################################################

        # self.camera.reparent_to(self.walker_controller.walker.root_np)
        # self.camera.set_pos(pos)
        # self.camera.look_at(self.floater)
        # self.camLens.set_fov(90)

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

        self.in_maze = False
        self.speed = Vec2(0)
        self.queue = deque()
        self.direction = Vec2(0)
        self.total_x = 0
        self.total_y = 0

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
        print('backward_pos', self.walker_controller.navigate(Vec3(0, 2, 3)) + walker_pos)
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


    def rotate_camera(self, angle):
        object_pos = self.diff
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

    def detect_camera_collision(self):
        walker_pos = self.walker_controller.get_walker_pos()
        camera_pos = self.camera_controller.get_pos()

        for con in self.world.contact_test(self.camera_controller.node(), use_filter=True).get_contacts():
            print(con.get_node1().get_name())
            mp = con.get_manifold_point()
            hit_pos = mp.get_position_world_on_b()
            degree, direction = self.get_rotation_angle(hit_pos, walker_pos, camera_pos)
            yield degree * direction

    def detect_obstruction(self):
        walker_pos = self.walker_controller.get_walker_pos()
        camera_pos = self.camera_controller.get_pos()
        backward_pos = self.walker_controller.navigate(Vec3(0, 4, 3)) + walker_pos

        if (result := self.world.ray_test_closest(
                camera_pos, walker_pos, mask=BitMask32.bit(4))).has_hit():
            if result.get_node().get_name() != 'maze_actor':

                degree, direction = self.get_rotation_angle(camera_pos, walker_pos, backward_pos)
                diff = self.rotate_camera(degree * direction)

                print('view obstruction', result.get_node().get_name(), camera_pos, walker_pos)
                return diff
        return None

    def control_camera(self, dt):
      
        # walker_pos = self.walker_controller.get_walker_pos()
        # camera_pos = self.camera_controller.get_pos(self.render)
       

        # print('backward_pos', self.walker_controller.navigate(Vec3(0, 4, 3)) + walker_pos)
        if angles := [angle for angle in self.detect_camera_collision()]:
            max_angle = max(angles)
            # self.camera_controller.set_pos(self.camera_controller.get_pos() + offset)
            diff = self.rotate_camera(max_angle)
            self.diff = diff

        # print(self.diff)
        pos = self.walker_controller.get_walker_pos() + self.diff
        self.camera_controller.set_pos(pos)

        
        if diff := self.detect_obstruction():
            self.diff = diff

            pos = self.walker_controller.get_walker_pos() + self.diff
            self.camera_controller.set_pos(pos)

        self.camera.look_at(self.walker_controller.walker.actor)

        # if (result := self.world.ray_test_closest(
        #         camera_pos, walker_pos, mask=BitMask32.bit(4))).has_hit():
        #     if result.get_node().get_name() != 'maze_actor':
     
        #         backward_pos = self.walker_controller.navigate(Vec3(0, 4, 3)) + walker_pos
        #         degree, direction = self.get_rotation_angle(camera_pos, walker_pos, backward_pos)
        #         next_pos = self.rotate_camera(degree * direction, walker_pos, camera_pos)
        #         self.camera_controller.set_pos(self.render, next_pos)
        #         self.camera.look_at(self.floater)

        # self.detect_camera_collision()
         
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
      

    def detect_camera_collision_2(self):
        walker_pos = self.walker_controller.get_walker_pos()
        camera_pos = self.camera_controller.get_pos()
        offset = Vec3()

        for con in self.world.contact_test(self.camera_controller.node(), use_filter=True).get_contacts():
            if con.get_node1().get_name().startswith('brick'):
                mp = con.get_manifold_point()
                # hit_pos = mp.get_position_world_on_b()
                # degree, direction = self.get_rotation_angle(hit_pos, walker_pos, camera_pos)
                # yield degree * direction

                dist = mp.get_distance()
                normal = mp.get_normal_world_on_b()
                offset -= normal * dist * 0.5
                print(con.get_node1().get_name(), dist, normal)

        print('------------------------------')
        return offset



    def control_camera_maze(self, dt, is_moving):
        walker_pos = self.walker_controller.get_walker_pos()

        
        # self.camera_controller.set_pos(pos)

        if space := self.walker_controller.check_current_pos():
            xy = self.maze_builder.space_to_cartesian(*space)
            self.direction = (xy - self.camera_controller.get_pos().xy).normalized()
            self.dest = xy
            self.total_x = abs(xy.x - self.camera_controller.get_pos().x)
            self.total_y = abs(xy.y - self.camera_controller.get_pos().y)

            
            # pos = Point3(xy, walker_pos.z + 3)
            # self.camera_controller.set_pos(pos)
            # self.camera.look_at(self.floater)

        if is_moving:
            distance = self.direction * 5 * dt

            self.total_x -= abs(distance.x)
            self.total_y -= abs(distance.y)

            if self.total_x < 0 or self.total_y < 0:
                pos = Point3(self.dest, walker_pos.z + 5)
                self.camera.look_at(self.floater)
            else:
                pos = Point3(self.camera_controller.get_pos().xy + distance, walker_pos.z + 5)
            self.camera_controller.set_pos(pos)

        



        # offset = self.detect_camera_collision_2()
        # print(offset)
        # self.diff += offset
        # pos = self.walker_controller.get_walker_pos() + self.diff
        # self.camera_controller.set_pos(pos)

        # print([ret for ret in self.detect_camera_collision_2()])
        # walker_pos = self.walker_controller.get_walker_pos()
        # camera_pos = self.camera_controller.get_pos()
        # space = Space(int(camera_pos.y), int(camera_pos.x))
        
        # if self.before_space != self.walker_controller.current_space:
         
        #     diff_x = self.walker_controller.current_space.col - self.before_space.col
        #     diff_y = self.walker_controller.current_space.row - self.before_space.row
        #     self.speed = Vec2(diff_x, diff_y)
        #     # print(Vec2(diff_x, diff_y))
        #     self.detect_camera_collision
        #     self.before_space = self.walker_controller.current_space

        # print(self.before_space, self.walker_controller.current_space)
        # print('speed', self.speed)
        # match tuple(self.speed):
        #     case (0, -1):
        #         # pos = Point3(self.before_space.col, walker_pos.y + 2, walker_pos.z + 3)
        #         if self.walker_controller.move_direction != 0:
        #             pos = Point3(self.before_space.col, camera_pos.y - dt * 5, walker_pos.z + 3)
        #             print(pos, walker_pos)
        #             self.camera_controller.set_pos(pos)
        #     case (0, 1):
        #         if self.walker_controller.move_direction != 0:
        #             pos = Point3(self.before_space.col, camera_pos.y + dt * 5, walker_pos.z + 3)
        #             print(pos, walker_pos)
        #             self.camera_controller.set_pos(pos)
        #     case (1, 0):
        #         if self.walker_controller.move_direction != 0:
        #             pos = Point3(camera_pos.x + dt * 5, self.before_space.row, walker_pos.z + 3)
        #             print(pos, walker_pos)
        #             self.camera_controller.set_pos(pos)

        #     case (-1, 0):
        #         if self.walker_controller.move_direction != 0:
        #             pos = Point3(camera_pos.x - dt * 5, self.before_space.row, walker_pos.z + 3)
        #             print(pos, walker_pos)
        #             self.camera_controller.set_pos(pos)

        # self.camera.look_at(self.floater)
        





    def control_walker(self, dt):
        motions = []
        moving = False

        if inputState.is_set('forward'):
            motions.append(Motions.FORWARD)
            moving = True
        if inputState.is_set('backward'):
            motions.append(Motions.BACKWARD)
            moving = True
        if inputState.is_set('left'):
            motions.append(Motions.LEFT)
        if inputState.is_set('right'):
            motions.append(Motions.RIGHT)

        self.walker_controller.update(motions, dt)
        return moving

    def update(self, task):
        dt = globalClock.get_dt()
        is_moving = self.control_walker(dt)

        if not self.in_maze:
            self.control_camera(dt)
            if self.walker_controller.is_walker_in_maze():
                # import pdb; pdb.set_trace()
                self.diff = Vec3(0, 2, 3)
                pos = self.walker_controller.get_walker_pos() + self.diff
                self.camera_controller.set_pos(pos)
                self.camera.look_at(self.floater)
                # pos = self.walker_controller.get_walker_pos()
                # self.camera_controller.set_pos(Point3(-18, 24, pos.z + 3))
                # self.before_space = self.walker_controller.current_space
                self.in_maze = True

        else:
            self.control_camera_maze(dt, is_moving)

        # self.aircraft_controller.update(dt)
        self.world.do_physics(dt)
        return task.cont


if __name__ == '__main__':
    app = Maze3D()
    app.run()
