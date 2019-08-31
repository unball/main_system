#!/usr/bin/env python
#from states.main_menu import MainMenu

from statics import static_classes
from strategy.movimentsDecider import Attacker
from strategy.movimentsDecider import MovimentsDecider

def start():
    state = MainMenu()
    while True:
        state.update()
        if state.QuitRequested:
            break
        if state.StateChangeRequested:
            state = state.next_state()

if __name__ == "__main__":
    #start()
    r = static_classes.world.robots[0]
    ent = Attacker()
    path = 0
    ent.possess(path, 0)
    r.entity.path = MovimentsDecider().shortestTragectory((0,0,0),(1,1,0), 0.01)
    





