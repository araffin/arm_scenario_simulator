from .gazeboObject import GazeboObject


class Table(GazeboObject):
    colorable_links = ['pocket', 'table']

    def __init__(self, name):
        GazeboObject.__init__(self, name)
        self._pressed = False

    def spawn(self, position, table_type="DREAM_table", orientation=None, **kwargs):
        return GazeboObject.spawn(self, table_type, position, orientation, **kwargs)

    def update_state(self, message):
        self._pressed = message.data == 1

    def set_table_color(self, rgba):
        self.set_color(rgba, 'table')
