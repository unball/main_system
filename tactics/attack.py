"""Defense tactic class module."""

from tactics.tactic import Tactic

from players import *


class Attack(Tactic):
    """Attack class child of Tactic()."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        super().__init__()

    def __indentify_formation(self):
        """Identify using fuzzy logic.

        In this case no logic is used, instantiating the formation always with
        one Driver, one Defender and one Goalkeeper.
        """
        self._formation = [Driver(), Defender(), Goalkeeper()]
        for index in range(len(self._formation)):
            self._formation[index].set_own_state(world.robots[index].pos,
                                                 world.robots[index].th,
                                                 world.robots[index].vel,
                                                 world.robots[index].w)

    @property
    def formation(self):
        """Return identified formation."""
        self.__indentify_formation()
        return self._formation


if __name__ == "__main__":
    teste = Attack()
    print(teste.find_formation())
