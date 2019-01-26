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
        self.tactic = self.coach.plan(self.world)
        zona_central = ZonaCentral(self.world)
        self.formation = self.tactic.find_formation(self.world)
        self.targets = TargetSelector(self.formation, self.world, zona_central)
        # TODO: arrumar arquitetura para garantir tipos diferentes de cálculo de targets
        
    def get_targets(self):
        """Getter of each robot target planned."""
        return self.targets
