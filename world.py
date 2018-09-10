"""World Module."""

from game_elements.ball import Ball
from game_elements.robot import Robot

MAGIC_NUMBER = 3
dummy_robot = {"pos": {"x": 0, "y": 0}, "th": 0,
               "vel": {"vx": 0, "vy": 0}, "w": 0}
dummy_ball = {"pos": {"x": 0, "y": 0}, "vel": {"vx": 0, "vy": 0}}


class World(object):
    """Docstring for the class."""

    def __init__(self, setting=0):
        """Init method."""
        if setting != 0:
            self.number_of_robots = setting['number_of_robots']
        else:
            self.number_of_robots = MAGIC_NUMBER
        self.robots = list(Robot() for robot in range(self.number_of_robots))
        self.ball = Ball()

    def update(self):
        """Follow the 'update()' methods from the element's classes."""
        # subscribe
        pass

    def dummy_update(self):
        """Temporary method with a false message from vision.For tests only."""
        for robot in self.robots:
            robot.update(dummy_robot['pos']['x'], dummy_robot['pos']['y'],
                         dummy_robot['th'])
        self.ball.update(dummy_ball['pos']['x'], dummy_robot['pos']['y'])

    def calc_velocities(self):
        """Docstring for the method."""
        pass

    @property
    def info(self):
        """Getter of the state of the world,including robots and ball."""
        return {"robots": self.robots, "ball": self.ball}


if __name__ == '__main__':
    world_state = World()
    world_state.dummy_update()
    print(world_state.robots[0].pos)
