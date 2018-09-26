"""First level fuzzy decider module."""


def error():
    """Print the standard error message for FIRST LEVEL FUZZY scope."""
    print("\nFIRST LEVEL FUZZY ERROR:")


class FirstLvlDecider(object):
    """Class docstring."""

    def __init__(self):
        """Initialize the class. Called when object is instantiated."""
        self.tactic = None

    def plan(self, world):
        """Docstring. Shall return a list of players."""
        self.world = world
        self.__identify_tactic()
        return self.tactic

    def __identify_tactic(self):
        """Identify the main tactic that should be used."""
        self.tactic = None
        # TODO: Attack() and Defense() examples


if __name__ == "__main__":
    pvt_method_test = FirstLvlDecider()
    pvt_method_test.__identify_tactic()
