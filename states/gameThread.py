from threading import Thread, Event
import gui.frameRenderer
import cv2
import time
import vision.cameras
import statics.configFile
import gui.mainWindow
import states.main_menu
import states.game_loop
import queue
from vision.mainVision.mainVision import MainVision
from strategy.strategy import Strategy
from controllers.ssRegulator import ssRegulator
from communication_system.radio_comm import RadioCommunicator

class GameThread():
    
    def __init__(self):
        # Private
        self._state = states.main_menu.MainMenu(self)
        self._events = queue.Queue()
        
        self._visionSystem = MainVision()
        self._strategySystem = Strategy()
        self._controlSystem = ssRegulator()
        self._radioComm = RadioCommunicator()
    
    def set_state(self, stateName):
        if stateName == "mainMenu":
            self._state.request_state_change(states.main_menu.MainMenu(self))
        elif stateName == "gameLoop":
            self._state.request_state_change(states.main_menu.GameLoop(self))
        else:
            pass
    
    @property
    def visionSystem(self):
        return self._visionSystem
    
    @property
    def strategySystem(self):
        return self._strategySystem
    
    @property
    def controlSystem(self):
        return self._controlSystem
    
    @property
    def radioComm(self):
        return self._radioComm
    
    def stop(self):
        self._state.request_quit()
    
    def run(self):
        self.thread = Thread(target=self.__loop__)
        self.thread.start()
        
    def addEvent(self, method, *args):
        self._events.put({"method": method, "args": args})
    
    def runQueuedEvents(self):
        while not self._events.empty():
            try:
                event = self._events.get_nowait()
                event["method"](*event["args"])
            except:
                pass
        
    def __loop__(self):
        while True:
            self.runQueuedEvents()
            self._state.update()
            if self._state.QuitRequested:
                break
            if self._state.StateChangeRequested:
                self._state = self._state.next_state()
            #time.sleep(1)
        
        
