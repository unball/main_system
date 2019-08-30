"""World Module."""

from statics.world.game_elements.ball import Ball
from statics.world.game_elements.robot import Robot
from statics import field

MAGIC_NUMBER = 3
dummy_robot = [{"pos": {"x": -.5, "y": 0}, "th": 0,
               "vel": {"vx": 0, "vy": .1}, "w": 0},
               {"pos": {"x": .73, "y": 0}, "th": 0,
               "vel": {"vx": 0, "vy": .1}, "w": 0},
               {"pos": {"x": -.50, "y": -.40}, "th": 0,
               "vel": {"vx": -.2, "vy": 0}, "w": 0}]
dummy_ball = {"pos": {"x": .1, "y": 0}, "vel": {"vx": 0, "vy": 0}}

class World(object):
    """Docstring for the class."""

    def __init__(self, setting=0):
        """Init method."""
        self.fieldSide = field.LEFT
        self.gameScore = 0
        self._isPaused = False
        if setting != 0:
            self._number_of_robots = setting['number_of_robots']
            self._field_x_length = setting['field_x_length']
            self._field_y_length = setting['field_y_length']
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
            for i in range(0,self._number_of_robots):
                self._robots[i] = self._robots[i].update(dummy_robot[i]['pos']['x'],
                                             dummy_robot[i]['pos']['y'],
                                             dummy_robot[i]['th'],
                                             dummy_robot[i]['vel']['vx'],
                                             dummy_robot[i]['vel']['vy'])

        except AttributeError:
            print("Tried to update internal 'robots' but list does not exist")
            return None
        try:
            self._ball.update(dummy_ball['pos']['x'], dummy_ball['pos']['y'])
        except AttributeError:
            print("Tried to update internal 'ball' but obj does not exist")
            return None

    def calc_velocities(self, time_to_derivate):
        """Docstring for the method."""
        for i in range(len(self.robots)):
            self.robots[i].calc_velocities(time_to_derivate)
        self.ball.calc_velocities(time_to_derivate)

    def change_field_side(self, new_side):
        print(new_side)
        self.fieldSide = new_side  

    def pause(self):
        self._isPaused = True

    def play(self):
        self._isPaused = False

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
        
    @property
    def field_x_length(self):
        return self._field_x_length
        
    @property
    def field_y_length(self):
        return self._field_y_length

    @property
    def isPaused(self):
        return self._isPaused
    
    def increaseGameScore(self):
        self.gameScore = self.gameScore + 1
    
    def decreaseGameScore(self):
        self.gameScore = self.gameScore - 1



if __name__ == '__main__':
    world_state = World()
    print(world_state.var)
    # world_state.dummy_update()
    # print("Robots:\n{}".format(world_state.robots))
    # print("Ball:\n{}".format(world_state.ball))
    # print("Number of robots:\n{}".format(world_state.number_of_robots))
