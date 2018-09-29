"""Movements module."""


class Movement(object):
    """Class docstring."""

    def __init__(self):
        """Init method."""
        self.target = None

    @property
    def target(self):
        """Mark an enemy player that has the ball, following it."""
        self._target = self.calc_target()
        return self._target
