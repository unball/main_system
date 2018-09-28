"""Driver module."""

from players.player import Player


class Driver(Player):
    """Class docstring."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        pass

    def calc_target(self):
        """Calculate it's own target based on world state."""
        self._target = [0, 0]
