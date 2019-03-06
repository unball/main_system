"""Defender module."""

from strategy.players.player import Player
from strategy.movements.mark import Mark


class Defender(Player):
    """Class docstring."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        self.id = 1

    def calc_target(self):
        """Calculate it's own target based on world state."""
        self.movement = Mark()
        print("Defender")
