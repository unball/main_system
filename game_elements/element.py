"""Element module."""


class Element(object):
    """Parent class of an element of game such as robot or ball."""

    def __init__(self):
        """Docstring for the method."""
        pass

    @property
    def inst_x(self):
        """Docstring for the method."""
        return self.__inst_x

    @inst_x.setter
    def inst_x(self, x):
        self.__inst_x = x

    @property
    def inst_y(self):
        """Docstring for the method."""
        return self.__inst_y

    @inst_y.setter
    def inst_y(self, y):
        self.__inst_y = y
