"""Goalkeeper module."""

from players.player import Player
from movements.goalkeep import Goalkeep


    
class Goalkeeper(Player):
    """Class docstring."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        self.id = 0
        pass

    def calc_target(self):
        """Calculate it's own target based on world state."""
        self.movement = Goalkeep()

    def __str__(self):
        return "Goalkeeper"
"""
alinhamento com o gol oponente
meio do campo pra frente


"""