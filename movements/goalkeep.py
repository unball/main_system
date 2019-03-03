"""Goalkeep module."""

from movements.movement import Movement

FIXED_X = 0.63
AREA_LENGHT = 0.5

class Goalkeep(Movement):
    """Docstring."""

    def __init__(self):
        """Init method."""
        pass

    def calc_target(self):
        """Calculate the instant target."""
        if self._ball.pos[1] > 0.25:
            self._target = [FIXED_X, 0.25]
        elif self._ball.pos[1] <= 0.25 and self._ball.pos[1] >= -0.25:
            self._target = [FIXED_X, self._ball.pos[1]]
        else:
            self._target = [FIXED_X, -0.25]
