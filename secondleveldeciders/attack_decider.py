"""Attack decider module."""

from secondleveldeciders.secondleveldecider import SecondLvlDecider

from players.striker import Striker
from players.goalkeeper import Goalkeeper
from players.defender import Defender

from membership_functions import *

magic_number = 14


class AttackDecider(SecondLvlDecider):
    """Class docstring."""

    def __init__(self):
        """Init method."""
        pass

    def id_formation(self, game_score):
        """Identify the formation based on the world state."""
        score = self.FUZZYscore(game_score)
        if type(score) == type([]):
            self.formation = score
            return None
        ball_x = magic_number * self.FUZZYball_x(self.world.ball.pos[0])
        total_score = ball_x + score
        self.formation = self.defuzzicator(total_score)

    def FUZZYscore(self, value):
        """Docstring."""
        if value <= -7 or value >= 7:
            return [Defender(), Striker(), Striker()]
        elif value > -7 and value <= -5:
            FA = i_neg(-7, -5, value)
            D = i_pos(-7, -5, value)
            return (FA * (-7)) + (D * (-5))
        elif value > -5 and value <= 0:
            D = i_neg(-5, 0, value)
            N = i_pos(-5, 0, value)
            return (D * (-5)) + (N * (0))
        elif value > 0 and value <= 5:
            N = i_neg(0, 5, value)
            A = i_pos(0, 5, value)
            return (N * (0)) + (A * (5))
        elif value > 5 and value <= 7:
            A = i_neg(5, 7, value)
            FA = i_pos(5, 7, value)
            return (A * (5)) + (FA * (7))

    def FUZZYball_x(self, value):
        """Docstring."""
        if value < -0.5:
            return -0.5
        elif value > 0.5:
            return 0.5
        elif value > -0.5 and value <= -0.25:
            SD = i_neg(-0.5, -0.25, value)
            D = i_pos(-0.5, -0.25, value)
            return (SD * (-0.5)) + (D * (-0.25))
        elif value > -0.25 and value <= 0:
            D = i_neg(-0.25, 0, value)
            N = i_pos(-0.25, 0, value)
            return (D * (-0.25)) + (N * (0))
        elif value > 0 and value <= 0.25:
            N = i_neg(0, 0.25, value)
            A = i_pos(0, 0.25, value)
            return (N * (0)) + (A * (0.25))
        elif value > 0.25 and value <= 0.5:
            A = i_neg(0.25, 0.5, value)
            SA = i_pos(0.25, 0.5, value)
            return (A * (0.25)) + (SA * (0.5))

    def defuzzicator(self, score):
        """Docstring."""
        if score <= -7:
            return [Goalkeeper(), Defender(), Defender()]
        elif score > -7 and score <= 3.5:
            return [Goalkeeper(), Defender(), Striker()]
        elif score > 3.5 and score <= 10.5:
            return [Goalkeeper(), Striker(), Striker()]
        else:
            # score > 10.5
            return [Defender(), Striker(), Striker()]

    def rearrange_formation(self):
        """Rearrange the list of players."""
        pass
