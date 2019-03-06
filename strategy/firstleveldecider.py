"""First level fuzzy decider module."""

from strategy.tactics.attack import Attack


def error():
    """Print the standard error message for FIRST LEVEL FUZZY scope."""
    print("\nFIRST LEVEL FUZZY ERROR:")


class FirstLvlDecider(object):  
    """Class docstring."""

    def __init__(self):
        """Initialize the class. Called when object is instantiated."""
        self.tactic = None

    def plan(self, world):
        """Shall return a Tactic instance."""
        self.world = world
        self.__identify_tactic()
        return self.tactic

    def __identify_tactic(self):
        """Identify the main tactic that should be used."""
        # Here should be the fuzzy logic (or other finding logic)
        # In this case no finding logic is used, always instantiating as Attack
        self.tactic = Attack()


if __name__ == "__main__":
    pvt_method_test = FirstLvlDecider()
    pvt_method_test.__identify_tactic()
