"""Player module."""


class Player(object):
    """Docstring."""

    def __init__(self):
        """Initialie the class attributes when instantiated."""
        pass

    def set_own_state(self, pos=None, th=None, vel=None, w=None):
        """Use info of the world state to set it's own pos, vel, etc.

        Hence, it's not necessary to each robot calculate "who it is".
        """
        self.pos, self.th, self.vel, self.w = pos, th, vel, w

    def get_target(self):
        """Return the target calculated within the player."""
        return self._target

    @property
    def target(self):
        """Return the target calculated for itself."""
        self.calc_target()
        return self.get_target()
