"""Strategy system module."""


class Strategy(object):
    """Class docstring."""

    def __init__(self):
        """Init method."""
        self.formation = None

    def plan(self, world):
        """Toplevel planner which contains all the deciders of the system."""
        try:
            self.formation = list(None for robot in world.number_of_robots)
        except TypeError:
            print("No number_of_robots set in World!")

    def get_targets(self):
        """Getter of each robot target planned."""
        pass
