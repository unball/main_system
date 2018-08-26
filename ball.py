class Ball(object):
    def __init__(self):
        pass

    @property
    def inst_x(self):
        return self.__inst_x

    @inst_x.setter
    def inst_x(self, x):
        self.__inst_x = x

    @property
    def inst_y(self):
        return self.__inst_y

    @y.setter
    def inst_y(self, y):
        self.__inst_y = y
