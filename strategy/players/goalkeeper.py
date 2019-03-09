"""Goalkeeper module."""

from strategy.players.player import Player
import numpy as np
# from strategy.movements.goalkeep import Goalkeep


class Goalkeeper(Player):
    """Class docstring."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        super().__init__()
        self.id = 0
        self._spin = 0

    def calc_target(self, world):
        """Calculate it's own target based on world state."""
        print("Goalkeeper")
        if np.linalg.norm(np.array(world.robots[0].pos) - np.array(world.ball.pos)) <= .07:
            if -1*world.robots[0].y*world.fieldSide >= 0:
                self._spin = -1
            else:
                self._spin = 1
        else:
            self._spin = 0
        xGoal = world.fieldSide * .75
        #testar velocidade minima (=.15?)
        if ((world.ball.vel[0]*world.fieldSide) > .1) and \
           ((world.ball.x*world.fieldSide)> .0):
           #verificar se a projeção está no gol
           #projetando vetor até um xGoal-> y = (xGoal-Xball) * Vyball/Vxball + yBall
           y =  (((xGoal-world.ball.x)/world.ball.vel[0])*world.ball.vel[1])+world.ball.y
           return [xGoal-.05*world.fieldSide, max(min(.23, y),-.23), np.pi/2]
        #Se não acompanha o y
        return [xGoal-.05*world.fieldSide, max(min(world.ball.y,.23),-.23), np.pi/2]

