"""Defender module."""

import numpy as np

from strategy.players.player import Player
# from strategy.movements.mark import Mark


class Defender(Player):
    """Class docstring."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        self.id = 1

    def calc_target(self, world):
        """Calculate it's own target based on world state."""
        print("Defender")
        ########definir z = [a**2, b**2]###########
        ballVel = np.array(world.ball.vel)
        ballPos = np.array(world.ball.pos)


        z = [.5**2, .3**2]
        Goal = np.array([-.7, 0])
        if (world.ball.vel[0]*world.fieldSide) > .15:
            c =  [sum(z*(ballVel)**2), 2*np.dot(z*ballVel, ballPos-Goal), sum(z*(ballPos-Goal)**2) - np.prod(z)]
            k = min(np.roots(c))
            if k.imag == 0:
                target = k*ballVel + ballPos
                if(not self.ballInsideArea(ballPos)):
                    target = list(target)
                    target.append(np.arctan2(target[1]-world.robots[1].pos[1], target[0]-world.robots[1].pos[0]))
                    return target
                target[1] = target[1] + .1
                target = list(target)
                target.append(np.arctan2(target[1]-world.robots[1].pos[1], target[0]-world.robots[1].pos[0]))
                return target        
        #apenas fica entre a bola e o Goal
        #if not ballInsideArea():
        signal = 1
        if ballPos[1]<0:
            ballPos[1] = ballPos[1]* -1
            signal = -1
        if sum(z*(ballPos-Goal)) > 0:
            target = list((np.sqrt(np.prod(z)/sum(z*(ballPos-Goal))) * (ballPos-Goal)) + Goal)
            target[1] =  target[1]*signal
            target.append(np.arctan2(target[1]-world.robots[1].pos[1], target[0]-world.robots[1].pos[0]))
            return target

        target = [(.15*world.fieldSide), (ballPos[1]+.1)]
        target.append(np.arctan2(target[1]-world.robots[1].pos[1], target[0]-world.robots[1].pos[0]))
        return target

    def ballInsideArea(self, ballPos):
        if abs(ballPos[0]) > .15:
            return False
        elif abs(ballPos[1]) > .35:
            return False
        return True
