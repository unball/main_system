"""Strategy system module."""

from strategy.firstleveldecider import FirstLvlDecider
from strategy.secondleveldeciders.attack_decider import AttackDecider

def error():
    """Print the standard error message for STRATEGY scope."""
    print("\nSTRATEGY ERROR:")


class Strategy(object):
    """Class docstring."""

    def __init__(self):
        """Init method."""
        self.firstLvlDec = FirstLvlDecider()
        self.tactic = None
        self.formation = None
        self.decider = AttackDecider()

    def plan(self, world):
        """Toplevel planner which contains all the deciders of the system."""
        self.world = world
        # TODO: VERIFICATION TEST FOR THE WORLD STATE
        self.tactic = self.firstLvlDec.plan(self.world)
        
        self.decider.setParams(world)
        self.decider.setFormation(world)
        self.targets = self.decider.updateTargets()
        

    def get_targets(self):
        """Getter of each robot target planned."""
        return self.targets
