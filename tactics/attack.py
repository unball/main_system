"""Defense tactic class module."""

from tactics.tactic import Tactic
from players.striker import Striker
from players.goalkeeper import Goalkeeper
from players.defender import Defender


class Attack(Tactic):
    """Attack class child of Tactic()."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        super().__init__()

    def indentify_formation(self):
        """Identify using fuzzy logic.

        In this case no logic is used, instantiating the formation always with
        one Striker, one Defender and one Goalkeeper.
        """
        self._formation = [Striker(), Striker(), Striker()]
        self.update_players_self_info()


if __name__ == "__main__":
    teste = Attack()
    print(teste.find_formation())
