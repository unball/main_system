"""Movements module."""


class Movement(object):
    """Class docstring."""

    def __init__(self):
        """Init method."""
        self._target = None

    @property
    def ball(self):
        """Property."""
        return self._ball

    @ball.setter
    def ball(self, __ball):
        """Setter ball."""
        self._ball = __ball

    @property
    def target(self):
        """Mark an enemy player that has the ball, following it."""
        self.calc_target()
        return self._target
