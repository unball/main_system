"""Striker module."""

from strategy.players.player import Player
# from strategy.movements.mark import Mark


#####################verify the robot number
import numpy as np

class Striker(Player):
    """Class docstring."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        self.id = 2

    def calc_target(self, world):
        """Calculate it's own target based on world state."""
        print("Striker")
        ballPos = np.array(world.ball.pos)
        robotPos = np.array(world.robots[2].pos)

        if (robotPos[0]-ballPos[0])*world.fieldSide>0:
            if abs(world.robots[2].th) != np.pi/2:
                yPro = abs((-.75*world.fieldSide) - robotPos[0])*np.tan(world.robots[2].th) + robotPos[1]
                if yPro > -.20 and yPro < .20:
                    ballYPro = np.array([-.75*world.fieldSide, yPro]) - ballPos
                    ballYProNorm = np.linalg.norm(ballYPro)
                    robotYPro = np.array([-.75*world.fieldSide, yPro]) - robotPos
                    robotYProNorm = np.linalg.norm(robotYPro)
                    if robotYProNorm*ballYProNorm != 0:
                        alpha = np.arccos(np.dot(robotYPro, ballYPro)/(robotYProNorm*ballYProNorm))
                        if alpha < np.pi/12 and abs(robotPos[0] - ballPos[0]) < .2:
                            target = [-7.5*world.fieldSide, 10*yPro]
                            target.append(np.arctan2(target[1]-world.robots[2].pos[1], target[0]-world.robots[2].pos[0]))
                            return target 

        target = world.ball.pos
        target.append(np.arctan2(target[1]-world.robots[2].pos[1], target[0]-world.robots[2].pos[0]))
        return target


if __name__ == "__main__":
    teste = Striker()
    teste.obstacles = list(2 for i in range(3))
    print(teste.obstacles)
