from abc import ABC, abstractmethod

class State(ABC):
    def __init__(self, thread):
        super().__init__()
        self.__quitRequested = False
        self.__stateChangeRequested = False
        self.__thread = thread
        self.__nextState = None

    @abstractmethod
    def update(self):
        pass 
    
    @property
    def QuitRequested(self):
        return self.__quitRequested

    @property
    def StateChangeRequested(self):
        return self.__stateChangeRequested
        
    @property
    def thread(self):
        return self.__thread

    def request_quit(self):
        self.__quitRequested = True

#    def request_state_change(self):
#        self.__stateChangeRequested = True

    def request_state_change(self, state):
        self.__stateChangeRequested = True
        self.__nextState = state

    def next_state(self):
        return self.__nextState


if __name__ == "__main__":
    pass
