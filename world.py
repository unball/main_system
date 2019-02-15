"""World Module."""

from game_elements.ball import Ball
from game_elements.robot import Robot

MAGIC_NUMBER = 3
dummy_robot = {"pos": {"x": 0, "y": 0}, "th": 0,
               "vel": {"vx": 0, "vy": 0}, "w": 0}
dummy_ball = {"pos": {"x": 0, "y": 0.25}, "vel": {"vx": 0, "vy": 0}}


class World(object):
    """Docstring for the class."""

    def __init__(self, setting=0):
        """Init method."""
        if setting != 0:
            self._number_of_robots = setting['number_of_robots']
        else:
            self._number_of_robots = None
            print("WARNING: no setting of world defined!")
            return None
        self._robots = list(Robot() for robot in range(self._number_of_robots))
        self._ball = Ball()
        print("World initiated successfully.")
        print("Number of robots: {}".format(self._number_of_robots))

    def update(self, vision_message):
        """Follow the 'update' methods from the element's classes."""
        found_list = list(vision_message.found)
        for i in range(len(found_list)):
            if found_list[i] is True:
                self._robots[i].update(vision_message.x[i],
                                       vision_message.y[i],
                                       vision_message.th[i])
        self._ball.update(vision_message.ball_x, vision_message.ball_y)

    def dummy_update(self):
        """Temporary method with a false message from vision.For tests only."""
        try:
            self._robots = list(robot.update(dummy_robot['pos']['x'],
                                             dummy_robot['pos']['y'],
                                             dummy_robot['th'])
                                for robot in self._robots)
        except AttributeError:
            print("Tried to update internal 'robots' but list does not exist")
            return None
        try:
            self._ball.update(dummy_ball['pos']['x'], dummy_ball['pos']['y'])
        except AttributeError:
            print("Tried to update internal 'ball' but obj does not exist")
            return None

    def calc_velocities(self):
        """Docstring for the method."""
        pass

    @property
    def info(self):
        """Getter of the state of the world,including robots and ball."""
        return {"robots": self._robots, "ball": self._ball}

    @property
    def robots(self):
        """self.robots property to improve access outside this scope."""
        try:
            return self._robots
        except AttributeError:
            print("Tried to access list 'self.robots' but list does not exist")
            return None

    @property
    def ball(self):
        """self.ball property to improve access outside this scope."""
        try:
            return self._ball
        except AttributeError:
            print("Tried to access 'ball' but obj does not exist.")
            return None

    @property
    def number_of_robots(self):
        """number_of_robots property to improve access outside this scope."""
        return self._number_of_robots


if __name__ == '__main__':
    world_state = World()
    print(world_state.var)
    # world_state.dummy_update()
    # print("Robots:\n{}".format(world_state.robots))
    # print("Ball:\n{}".format(world_state.ball))
    # print("Number of robots:\n{}".format(world_state.number_of_robots))
