from states.state import State
from abc import ABC

from strategy.strategy import Strategy
from controllers.ssRegulator import ssRegulator
from communication.radio_comm import RadioCommunicator
from statics import static_classes

class GameLoop(State):
    def __init__(self):
        super().__init__()
        self.__strategySystem = Strategy()
        self.__controlSystem = ssRegulator()
        self.__radioComm = RadioCommunicator()

    def update(self):
        # Vision System
        # Measurement System
        self.__strategySystem.plan(static_classes.world)
        targets, spin = self.__strategySystem.get_targets()
        velocities = self.__controlSystem.actuate(targets, static_classes.world)
        self.__radioComm.send(velocities)
        # Communication

    def next_state(self):
        pass

if __name__ == "__main__":
    pass