import sys
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode
from panda3d.core import Vec3
from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock

from create_maze_3d import MazeBuilder3D
from lights import BasicAmbientLight, BasicDayLight
from scene import Ground


class Maze3D(ShowBase):

    def __init__(self):
        super().__init__()
        self.disable_mouse()
        self.world = BulletWorld()
        self.world.set_gravity(Vec3(0, 0, -9.81))

        self.debug = self.render.attach_new_node(BulletDebugNode('debug'))
        self.world.set_debug_node(self.debug.node())

        self.creator = MazeBuilder3D(self.world)
        self.creator.build(21, 21)

        # self.camera.set_pos(30, -30, 50)
        self.camera.set_pos(0, 0, 80)
        self.camera.look_at(0, 0, 0)

        self.ambient_light = BasicAmbientLight()
        self.ambient_light.reparent_to(self.render)
        self.day_light = BasicDayLight()
        self.day_light.reparent_to(self.render)
        ground = Ground()
        ground.reparent_to(self.render)
        self.world.attach(ground.node())

        self.accept('escape', sys.exit)
        self.accept('d', self.toggle_debug)
        self.taskMgr.add(self.update, 'update')

    def toggle_debug(self):
        if self.debug.is_hidden():
            self.debug.show()
        else:
            self.debug.hide()

    def update(self, task):
        dt = globalClock.get_dt()

        self.world.do_physics(dt)
        return task.cont


if __name__ == '__main__':
    app = Maze3D()
    app.run()
