import rospy
from arm_scenario_simulator.msg import Int8Stamped

from .gazeboObject import GazeboObject


class Lever(GazeboObject):

    def __init__(self, name):
        self.colorable_links = ['base', 'lever']
        GazeboObject.__init__(self, name)
        self._pushed = None
        rospy.Subscriber("/" + name + "/is_pushed", Int8Stamped, self.update_state)

    def spawn(self, position, orientation=None, **extra):
        return GazeboObject.spawn(self, 'DREAM_lever', position, orientation, **extra)

    def update_state(self, message):
        self._pushed = message.data == 1

    def is_pushed(self):
        return self._pushed

    def set_base_color(self, rgba):
        self.set_color(rgba, 'base')

    def set_lever_color(self, rgba):
        self.set_color(rgba, 'lever')
