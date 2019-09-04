from states.state import State
from abc import ABC

from strategy.strategy import Strategy
from controllers.ssRegulator import ssRegulator
from communication_system.radio_comm import RadioCommunicator
from statics import static_classes
from vision.mainVision.mainVision import MainVision
import time

class GameLoop(State):
    def __init__(self, thread):
        super().__init__(thread)

    def update(self):
        # Vision System
        t0 = time.time()
        self.thread.visionSystem.update()
        self.thread.strategySystem.plan()
        targets, spin = self.thread.strategySystem.get_targets()
        velocities = self.thread.controlSystem.actuate(targets, static_classes.world)
        self.thread.radioComm.send(velocities)
        print(time.time()-t0)

#    def next_state(self):
#        pass

if __name__ == "__main__":
    pass
