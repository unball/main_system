"""Defense tactic class module."""

from tactics.tactic import Tactic


class Attack(Tactic):
    """Attack class child of Tactic()."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        super().__init__()
