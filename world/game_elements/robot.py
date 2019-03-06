"""Robot class module."""

from world.game_elements.element import Element


class Robot(Element):
    """Robot class child of Element()."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        super().__init__()


if __name__ == "__main__":
    robot = Robot()
