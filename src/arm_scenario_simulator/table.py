import rospy
from .gazeboObject import GazeboObject
from std_msgs.msg import ColorRGBA, Int8
from arm_scenario_simulator.msg import MaterialColor, Int8Stamped
from .parameters import COLOR_TYPE


class Table(GazeboObject):
    colorable_links = ['pocket', 'table']

    def __init__(self, name):
        GazeboObject.__init__(self, name)
        self._pressed = False

    def spawn(self, position, orientation=None, **extra):
        return GazeboObject.spawn(self, 'DREAM_table', position, orientation, **extra)

    def update_state(self, message):
        self._pressed = message.data == 1

    def set_table_color(self, rgba):
        self.set_color(rgba, 'table')
