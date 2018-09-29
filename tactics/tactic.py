"""Tactic module."""


def fit_obstacles(index, obstacles):
    """Remove the robot itself from a list of 'robotic' obstacles received."""
    obstacle_tobe_rmvd = obstacles[index]
    obstacles.remove(obstacle_tobe_rmvd)
    return obstacles


class Tactic(object):
    """Docstring."""

    def __init__(self):
        """Init method."""
        self._formation = None

    def find_formation(self, world=None):
        """Shall return a list of players."""
        self.world = world
        formation = self.formation
        return formation

    def update_players_self_info(self):
        """Update each player with the world info necessary for it's scope."""
        obstacles = list(self.world.robots)
        # obstacles is a list of the position of all robots
        for index in range(len(self._formation)-1):
            _obstacles = fit_obstacles(index, obstacles)
            # it's own position is removed from the list of obstacles
            self._formation[index].set_own_state(self.world.robots[index].pos,
                                                 self.world.robots[index].th,
                                                 self.world.robots[index].vel,
                                                 self.world.robots[index].w)
            self._formation[index].obstacles = _obstacles
            self._formation[index].ball = self.world.ball
            _obstacles = obstacles

    @property
    def formation(self):
        """Return identified formation."""
        self.indentify_formation()
        return self._formation
