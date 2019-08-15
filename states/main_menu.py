from state import State
from abc import ABC

from game_loop import GameLoop

class MainMenu(State):
    def __init__(self):
        super().__init__()

    def update(self):
        # Verificar Input Manager
        pass

    def next_state(self):
        return GameLoop()

if __name__ == "__main__":
    pass