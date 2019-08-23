from threading import Thread, Event
from gi.repository import GLib
import gui.frameRenderer
import cv2
import time
import vision.cameras
import statics.configFile
import gui.mainWindow
import states.main_menu
import states.game_loop
from vision.mainVision.mainVision import MainVision
from strategy.strategy import Strategy
from controllers.ssRegulator import ssRegulator
from communication_system.radio_comm import RadioCommunicator

class GameThread():
    
    def __init__(self):
        # Private
        self.state = states.main_menu.MainMenu(self)
        
        self.visionSystem = MainVision()
        self.strategySystem = Strategy()
        self.controlSystem = ssRegulator()
        self.radioComm = RadioCommunicator()
    
    def set_state(self, stateName):
        if stateName == "mainMenu":
            self.state.request_state_change(states.main_menu.MainMenu(self))
        elif stateName == "gameLoop":
            self.state.request_state_change(states.main_menu.GameLoop(self))
        else:
            pass
    
    def stop(self):
        self.state.request_quit()
    
    def run(self):
        self.thread = Thread(target=self.__loop__)
        self.thread.start()
        
    def __loop__(self):
        while True:
            self.state.update()
            if self.state.QuitRequested:
                break
            if self.state.StateChangeRequested:
                self.state = self.state.next_state()
            #time.sleep(1)
        
        
