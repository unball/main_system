"""Control system module."""


class Controller():
    """Class docstring."""

    def __init__(self):
        """Init method."""
        pass

    def actuate(self, references, world):
        """Control system actuator itself. Receives references and world info."""
        output_vel = self.regulate(references, world)
        return output_vel
