#!/usr/bin/env python
from main_menu import MainMenu

def start():
    state = MainMenu()
    while True:
        state.update()
        if state.QuitRequested:
            break

if __name__ == "__main__":
    start()





