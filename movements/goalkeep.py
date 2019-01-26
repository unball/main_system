"""Goalkeep module."""

from movements.movement import Movement
import numpy as np
from world import World


FIXED_X = 0.63
AREA_LENGHT = 0.5

class Goalkeep(Movement):
    """Docstring."""

    def __init__(self):
        """Init method."""
        pass

    def calc_target(self):
        """Calculate the instant target."""
        print("shoot: ", self.shoot())
        if self._ball.pos[1] > 0.25:
           self._target = [FIXED_X, 0.25]
        elif self._ball.pos[1] <= 0.25 and self._ball.pos[1] >= -0.25:
            self._target = [FIXED_X, self._ball.pos[1]]
        else:
            self._target = [FIXED_X, -0.25]
"""
    def shoot(self):
        ball_vec = np.array(World.ball)
        af = Strategy.formation
        for i in range(3):
            if af[i].id == 0:
                 goalkeep_vec = np.array(World.robot[i])
        sub = (ball_vec-goalkeep_vec)
        if sub[0] == 0:
            return False
        else:
            m = sub[1]/sub[0]
            proj = goalkeep_vec[1] + m * (75 - goalkeep_vec[0]) #verificar o lado
            if abs(proj) <= 20: 
                return True    
        return False


"""
      





