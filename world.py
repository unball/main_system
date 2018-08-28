"""World Module."""

from game_elements.ball import Ball
from game_elements.robot import Robot



class World(object):
    """Docstring for the class."""

    def __init__(self, formation):
        """Blank init."""
        pass

    def update(self):
        """Docstring for the method."""
        self.past_info = self.instant_info
        # self.instant_info = publisher
        pass

    def get_inst_info(self):
        """Docstring for the method."""
        return self.instant_info

    def calc_velocities(self):
        """Docstring for the method."""
        pass
