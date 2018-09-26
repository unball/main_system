"""Strategy system module."""

from firstleveldecider import FirstLvlDecider


def error():
    """Print the standard error message for STRATEGY scope."""
    print("\nSTRATEGY ERROR:")


class Strategy(object):
    """Class docstring."""

    def __init__(self):
        """Init method."""
        self.coach = FirstLvlDecider()
        self.tactic = None
        self.formation = None

    def plan(self, world):
        """Toplevel planner which contains all the deciders of the system."""
        self.world = world
        # TODO: VERIFICATION TEST FOR THE WORLD STATE
        self.tactic = self.coach.plan(world)
        self.formation = self.tactic.(world)

    def get_targets(self):
        """Getter of each robot target planned."""
        pass
