"""Strategy system module."""

from firstleveldecider import FirstLvlDecider
from target_selector.standard_selector import StandardSelector


def error():
    """Print the standard error message for STRATEGY scope."""
    print("\nSTRATEGY ERROR:")


class Strategy(object):
    """Class docstring."""

    def __init__(self):
        """Init method."""
        self.FirstLvlDecider = FirstLvlDecider()
        self.selector = StandardSelector()
        self.tactic = None
        self.formation = None

    def plan(self, world):
        """Toplevel planner which contains all the deciders of the system."""
        self.world = world
        # TODO: VERIFICATION TEST FOR THE WORLD STATE
        self.tactic = self.FirstLvlDecider.plan(self.world)
        # zona_central = ZonaCentral(self.world)
        self.formation = self.tactic.find_formation(self.world)
        self.selector.update(self.formation, self.world)
        self.targets = self.selector.calcTargets()

    def get_targets(self):
        """Getter of each robot target planned."""
        return self.targets
