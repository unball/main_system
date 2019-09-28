from states.state import State
from abc import ABC

from strategy.strategy import Strategy
from controllers.ssRegulator import ssRegulator
from communication.radio_comm import RadioCommunicator
from statics import static_classes
from vision.mainVision.mainVision import MainVision
import time
from statics.static_classes import world
import numpy as np

class GameLoop(State):
    def __init__(self, thread):
        super().__init__(thread)

    def update(self):
        # Vision System
        t0 = time.time()
        self.thread.visionSystem.update()
        #TODO: calcular o tempo de loop pra derivacao das posicoes
        world.calc_velocities(0.03) # <----- ERRADO
        self.thread.strategySystem.plan()
        targets, spin = self.thread.strategySystem.get_targets()
        targets = [(-0.56,world.ball.y,np.pi/2),(0,0,0),(0,0,0)]
        velocities = self.thread.controlSystem.actuate(targets, static_classes.world)
        print(velocities[0])


        self.thread.radioComm.send(velocities)
        #print(time.time()-t0)

#    def next_state(self):
#        pass

if __name__ == "__main__":
    pass
