"""Ball class module."""

from world.game_elements.element import Element


class Ball(Element):
    """Ball class child of Element()."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        super().__init__()


if __name__ == "__main__":
    ball = Ball()
    print("Testing")
