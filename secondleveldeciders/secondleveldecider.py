"""Second level decider."""


class SecondLvlDecider(object):
    """Class docstring."""

    def __init__(self):
        """Init method."""
        pass

    def get_formation(self):
        """Return the formation calculated."""
        self.id_formation(self._game_score)
        self.rearrange_formation(self.world)
        return self.formation


    def update_world(self, world):
        """Update the world for this scope of decision."""
        self.world = world

    @property
    def game_score(self):
        """Docstring."""
        return self._game_score

    @game_score.setter
    def game_score(self, __game_score):
        """Docstring."""
        self._game_score = __game_score