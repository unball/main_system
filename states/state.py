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
    def QuitRequested(self):
        return self.__quitRequested

    @property
    def StateChangeRequested(self):
        return self.__stateChangeRequested

    def request_quit(self):
        self.__quitRequested = True

    def request_state_change(self):
        self.__stateChangeRequested = True


if __name__ == "__main__":
    pass