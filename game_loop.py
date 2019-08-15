from state import State
from abc import ABC

from strategy.strategy import Strategy
class GameLoop(State):
    def __init__(self):
        super().__init__()
        self.__strategySystem = Strategy()

    def update(self):
        # Vision System
        # Measurement System
        self.__strategySystem.plan(static_classes.world)
        targets, spin = self.__strategySystem.get_targets()
        # Communication System
        pass

    def next_state(self):
        pass

if __name__ == "__main__":
    pass