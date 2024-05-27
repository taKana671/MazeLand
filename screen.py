import direct.gui.DirectGuiGlobals as DGG
from direct.gui.DirectFrame import DirectFrame
from direct.gui.DirectLabel import DirectLabel
from direct.gui.DirectButton import DirectButton
from panda3d.core import CardMaker, LColor
from direct.interval.IntervalGlobal import Sequence, Func


class Frame(DirectFrame):

    def __init__(self, parent):
        super().__init__(parent=parent)
        self.initialiseoptions(type(self))


class Label(DirectLabel):

    def __init__(self, frame, text, pos, font, text_scale=0.2):
        super().__init__(
            parent=frame,
            text=text,
            pos=pos,
            text_fg=LColor(1, 1, 1, 1),
            text_font=font,
            text_scale=text_scale,
            frameColor=LColor(1, 1, 1, 0)
        )
        self.initialiseoptions(type(self))


class Button(DirectButton):

    def __init__(self, frame, text, pos, font, command=None, text_scale=0.12):
        super().__init__(
            parent=frame,
            relief=None,
            frameSize=(-0.3, 0.3, -0.07, 0.07),
            text=text,
            pos=pos,
            text_fg=LColor(1, 1, 1, 1),
            text_scale=text_scale,
            text_font=font,
            text_pos=(0, -0.04),
            command=command
        )
        self.initialiseoptions(type(self))


class Screen:

    def __init__(self, frame):
        self.frame = frame
        self.color_in = LColor(0, 0, 0, 0.9)
        self.color_out = LColor(0, 0, 0, 0)
        self.create_screen()

    def create_screen(self):
        cm = CardMaker('card')
        cm.set_frame_fullscreen_quad()
        self.background = base.render2d.attach_new_node(cm.generate())
        self.background.set_transparency(1)
        self.background.set_color(self.color_in)

    def fade_out(self, callback, *args, **kwargs):
        Sequence(
            Func(self.frame.hide),
            self.background.colorInterval(1.0, self.color_out),
            Func(callback, *args, **kwargs)
        ).start()

    def fade_in(self, callback, *args, **kwargs):
        Sequence(
            self.background.colorInterval(1.0, self.color_in),
            Func(self.gui.show),
            Func(callback, *args, **kwargs)
        ).start()

    def hide(self):
        self.frame.hide()
        self.background.hide()

    def show(self):
        self.frame.show()
        self.background.show()