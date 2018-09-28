"""Tactic module."""


class Tactic(object):
    """Docstring."""

    def __init__(self):
        """Init method."""
        self._formation = None

    def find_formation(self, world=None):
        """Shall return a list of players."""
        if world is None:
            print("WARNING! No 'world' passed as argument.")
        self.world = world
        return self.formation
