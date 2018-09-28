"""Defense tactic class module."""

from tactics.tactic import Tactic

from players import *

def fit_obstacles(index, obstacles):
    obstacle_tobe_rmvd = obstacles[index]
    obstacles.remove(obstacle_tobe_rmvd)
    return obstacles


class Attack(Tactic):
    """Attack class child of Tactic()."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        super().__init__()

    def indentify_formation(self):
        """Identify using fuzzy logic.

        In this case no logic is used, instantiating the formation always with
        one Driver, one Defender and one Goalkeeper.
        """
        self._formation = [Driver(), Defender(), Goalkeeper()]
        self.update_players_self_info()

    def update_players_self_info(self):
        """Update each player with the world info necessary for it's scope."""
        obstacles = list(self.world.robots)
        # obstacles is a list of the position of all robots
        for index in range(len(self._formation)):
            _obstacles = fit_obstacles(index, obstacles)
            # it's own position is removed from the list of obstacles
            self._formation[index].set_own_state(self.world.robots[index].pos,
                                                 self.world.robots[index].th,
                                                 self.world.robots[index].vel,
                                                 self.world.robots[index].w)
            self._formation[index].obstacles = _obstacles
            self._formation[index].ball = self.world.ball
            _obstacles = obstacles


if __name__ == "__main__":
    teste = Attack()
    print(teste.find_formation())
