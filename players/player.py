"""Player module."""


class Player(object):
    """Docstring."""

    def __init__(self):
        """Initialie the class attributes when instantiated."""
        self.__obstacles = None
        self._target = None

    def set_own_state(self, pos=None, th=None, vel=None, w=None):
        """Use info of the world state to set it's own pos, vel, etc.

        Hence, it's not necessary to each robot calculate "who it is".
        """
        self.pos, self.th, self.vel, self.w = pos, th, vel, w

    @property
    def obstacles(self):
        """Get a list of obstacles to be avoided."""
        return self.__obstacles

    @obstacles.setter
    def obstacles(self, _obstacles):
        """Set a list of obstacles to be avoided."""
        self.__obstacles = _obstacles

    @property
    def target(self):
        """Get the target calculated for itself."""
        self.calc_target()
        return self.get_target()

    @property
    def ball(self):
        """Define the property so the setter can be used."""
        return self.ball

    @ball.setter
    def ball(self, ball):
        """Setter of ball position on each robot scope."""
        self.ball = ball
