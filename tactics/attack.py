"""Defense tactic class module."""

from tactics.tactic import Tactic
#from players.striker import Striker
#from players.goalkeeper import Goalkeeper
#from players.defender import Defender

GAME_SCORE = 8
# Our Score - enemey score


class Attack(Tactic):
    """Attack class child of Tactic()."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        super().__init__()
        self.formation1 = None

    def indentify_formation(self):
        """Identify using fuzzy logic.

        In this case no logic is used, instantiating the formation always with
        one Striker, one Defender and one Goalkeeper.
        """
        self.decider.update_world(self.world)
        self.decider.game_score = GAME_SCORE #tem que vir do world
        self._formation = self.decider.get_formation()
        self.formation1 = self._formation
        #self._formation = [Goalkeeper(), Defender(), Striker()]
        self.update_players_self_info()


if __name__ == "__main__":
    teste = Attack()
    print(teste.find_formation())
