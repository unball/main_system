"""Tactic module."""


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

    @property
    def formation(self):
        """Return identified formation."""
        self.indentify_formation()
        return self._formation
