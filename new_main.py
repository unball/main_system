#!/usr/bin/env python
from states.main_menu import MainMenu

def start():
    state = MainMenu()
    while True:
        state.update()
        if state.QuitRequested:
            break
        if state.StateChangeRequested:
            state = state.next_state()

if __name__ == "__main__":
    start()





