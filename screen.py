import direct.gui.DirectGuiGlobals as DGG
from direct.gui.DirectFrame import DirectFrame
from direct.gui.DirectLabel import DirectLabel
from direct.gui.DirectButton import DirectButton
from panda3d.core import CardMaker, LColor
from direct.interval.IntervalGlobal import Sequence, Func


class Frame(DirectFrame):

    def __init__(self):
        super().__init__(parent=base.aspect2d)
        self.initialiseoptions(type(self))
        self.buttons = []

    def display(self, show=True):
        if show:
            self.reparent_to(base.aspect2d)
        else:
            self.detach_node()


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

    def __init__(self, frame, text, pos, font, command=None, text_scale=0.12, focus=False):
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
        self.frame = frame
        self.initialiseoptions(type(self))

        self.focus_color = (1, 1, 1, 1)
        self.blur_color = (1, 1, 1, 0.5)
        self.is_focus = True
        self.frame.buttons.append(self)

        if not focus:
            self.blur()

        self.bind(DGG.ENTER, self.roll_over)
        self.bind(DGG.EXIT, self.roll_out)

    def roll_over(self, param=None):
        self.focus()

        for btn in self.frame.buttons:
            if btn != self and btn.is_focus:
                btn.blur()
                break

    def roll_out(self, param=None):
        if all(not btn.is_focus for btn in self.frame.buttons if btn != self):
            return
        self.blur()

    def focus(self):
        self.is_focus = True
        self.colorScaleInterval(0.05, self.focus_color, blendType='easeInOut').start()

    def blur(self):
        self.is_focus = False
        self.colorScaleInterval(0.05, self.blur_color, blendType='easeInOut').start()


class Screen:

    def __init__(self, default_frame):
        self.frame = default_frame

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
            Func(self.frame.display, False),
            self.background.colorInterval(1.0, self.color_out),
            Func(self.background.detach_node),
            Func(callback, *args, **kwargs)
        ).start()

    def fade_in(self, callback, *args, **kwargs):
        Sequence(
            Func(self.background.reparent_to, base.render2d),
            self.background.colorInterval(1.0, self.color_in),
            Func(self.frame.display, True),
            Func(callback, *args, **kwargs)
        ).start()