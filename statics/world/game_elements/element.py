"""Element module."""

alpha = 0.1

class Element(object):
    """Parent class of an element of game such as robot or ball."""

    def __init__(self):
        """Docstring for the method."""
        self.inst_x = 0
        self.inst_y = 0
        self.inst_th = 0

        self.prev_x = 0
        self.prev_y = 0
        self.prev_th = 0

        self.inst_vx = 0
        self.inst_vy = 0
        self.inst_w = 0

    def __repr__(self):
        """Docstring for the method."""
        x = 'x: ' + str(self.inst_x) + '\n'
        y = 'y: ' + str(self.inst_y) + '\n'
        th = 'th: ' + str(self.inst_th) + '\n'
        vx = 'vx: ' + str(self.inst_vx) + '\n'
        vy = 'vy: ' + str(self.inst_vy) + '\n'
        w = 'w: ' + str(self.inst_w)
        info = x + y + th + vx + vy + w
        return info

    def update(self, x=0, y=0, th=0):
        """Docstring for the method."""
        self.prev_x = self.inst_x
        self.prev_y = self.inst_y
        self.prev_th = self.inst_th
        self.inst_x = x
        self.inst_y = y
        self.inst_th = th
        return self

    @property
    def pos(self):
        """Getter property of self.inst_pos."""
        return [self.inst_x, self.inst_y]

    @pos.setter
    def pos(self, x, y):
        self.inst_x = x
        self.inst_y = y

    @property
    def th(self):
        """Getter property of self.inst_th."""
        return self.inst_th

    @th.setter
    def th(self, th):
        self.inst_th = th

    @property
    def vel(self):
        """Getter property of self.inst_vel."""
        return [self.inst_vx, self.inst_vy]

    @vel.setter
    def vel(self, vx, vy):
        self.inst_vx = vx
        self.inst_vy = vy

    @property
    def w(self):
        """Getter property of self.inst_w."""
        return self.inst_w

    @w.setter
    def w(self, w):
        self.inst_w = w

    def calc_velocities(self, time_to_derivate):
        """Calcalate the velocities with instant and previous positions."""
        self.inst_vx = ((self.inst_x - self.prev_x)/time_to_derivate)*alpha + (self.inst_vx)*(1-alpha)
        self.inst_vy = ((self.inst_y - self.prev_y)/time_to_derivate)*alpha + (self.inst_vy)*(1-alpha)
        self.inst_w = ((self.inst_th - self.prev_th)/time_to_derivate)*alpha + (self.inst_w)*(1-alpha)

    @property
    def x(self):
        return self.inst_x

    @property
    def y(self):
        return self.inst_y


if __name__ == "__main__":
    element = Element()
