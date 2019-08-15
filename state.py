from abc import ABC, abstractmethod

class State(ABC):
    def __init__(self):
        super().__init__()
        self.__quitRequested = False
        self.__stateChangeRequested = False

    @abstractmethod
    def update(self):
        pass 

    @abstractmethod    
    def next_state(self):
        pass
    @property
    def quit_requested(self):
        return self.quit_requested


if __name__ == "__main__":
    pass