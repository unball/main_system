"""Strategy system module."""

from strategy.firstleveldecider import FirstLvlDecider
from strategy.secondleveldeciders.attack_decider import AttackDecider
from strategy.players.goalkeeper import Goalkeeper
from strategy.players.striker import Striker
from strategy.players.defender import Defender

def error():
    """Print the standard error message for STRATEGY scope."""
    print("\nSTRATEGY ERROR:")


class Strategy(object):
    """Class docstring."""

    def __init__(self):
        """Init method."""
        self.firstLvlDec = FirstLvlDecider()
        self.tactic = None
        self.formation = [Goalkeeper(), Defender(), Striker()]
        self.decider = AttackDecider()

    def plan(self, world):
        """Toplevel planner which contains all the deciders of the system."""
        self.world = world
        # TODO: VERIFICATION TEST FOR THE WORLD STATE

        # # Fuzzy
        # self.tactic = self.firstLvlDec.plan(self.world)
        # self.decider.setParams(world)
        # self.decider.setFormation(world)
        # self.targets = self.decider.updateTargets()
        self.targets = list(player.calc_target(self.world) for player in self.formation)

    def get_targets(self):
        """Getter of each robot target planned."""
        return self.targets
