"""Mark module."""

from strategy.movements.movement import Movement


class Mark(Movement):
    """Class docstring."""

    def __init__(self):
        """Init method."""
        pass

    def calc_target(self):
        """Calculate the instant target."""
        self._target = self._ball.pos
