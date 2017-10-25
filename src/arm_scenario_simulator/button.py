import rospy
from .gazeboObject import GazeboObject
from arm_scenario_simulator.msg import Int8Stamped


class Button(GazeboObject):

    def __init__(self, name):
        GazeboObject.__init__(self, name)
        self._pressed = False
        self.colorable_links = ['base', 'button']
        rospy.Subscriber("/" + name + "/is_pressed", Int8Stamped, self.update_state)

    def spawn(self, position, button_type="DREAM_push_button", orientation=None, **kwargs):
        return GazeboObject.spawn(self, button_type, position, orientation, **kwargs)

    def update_state(self, message):
        self._pressed = message.data == 1

    def is_pressed(self):
        return self._pressed

    def set_base_color(self, rgba):
        self.set_color(rgba, 'base')

    def set_button_color(self, rgba):
        self.set_color(rgba, 'button')
