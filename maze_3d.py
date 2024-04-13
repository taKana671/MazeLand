import sys
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode
from panda3d.core import Vec3
from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock


class Maze3D(ShowBase):

    def __init__(self):
        super().__init__()
        self.world = BulletWorld()
        self.world.set_gravity(Vec3(0, 0, -9.81))

        self.accept('escape', sys.exit)
        self.taskMgr.add(self.update, 'update')

    def update(self, task):
        dt = globalClock.get_dt()

        self.world.do_physics(dt)
        return task.cont


if __name__ == '__main__':
    app = Maze3D()
    app.run()
