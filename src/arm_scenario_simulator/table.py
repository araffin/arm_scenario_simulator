from .gazeboObject import GazeboObject


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
