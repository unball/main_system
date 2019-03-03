"""Striker module."""

from players.player import Player
from movements.mark import Mark


class Striker(Player):
    """Class docstring."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        self.id = 2

    def calc_target(self):
        """Calculate it's own target based on world state."""
        self.movement = Mark()
        print("Striker")
        # Here, self._target is supposed to be instantiated with a Movement


if __name__ == "__main__":
    teste = Striker()
    teste.obstacles = list(2 for i in range(3))
    print(teste.obstacles)
