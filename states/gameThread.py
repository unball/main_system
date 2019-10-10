from threading import Thread, Event
import gui.frameRenderer
import cv2
import time
import vision.cameras
import statics.configFile
import gui.mainWindow
import states.config_vision
import states.game_loop
import states.config_strategy
import states.config_world
import queue
import traceback
from gui.guiMethod import guiMethod
from vision.mainVision.mainVision import MainVision
from strategy.strategy import Strategy
from controllers.ssRegulator import ssRegulator, nonLinearControl
from comm.radio_comm import RadioCommunicator
from statics.static_classes import world
import signal
import roshandler.roshandler as rh

class GameThread():
    
    def __init__(self, initialState=states.game_loop.UiGameLoop):
        # Private
        self._state = initialState(self)
        self._events = queue.Queue()
        
        self._visionSystem = MainVision(self)
        self._strategySystem = Strategy(self)
        self._controlSystem = nonLinearControl(self)
        self._radioComm = RadioCommunicator(self)
        
        
        self._loop_time = 0
        self._processing_time = 0

        signal.signal(signal.SIGINT, self.exit)
        
    def compute_average(self, v0, v, p=0.1):
        return v0*(1-p) + v*p
    
    @guiMethod
    def update_stats(self, processing_time, loop_time):
        self._loop_time = self.compute_average(self._loop_time, loop_time)
        self._processing_time = self.compute_average(self._processing_time, processing_time)
        gui.mainWindow.MainWindow().getObject("stats_label").set_text("Tempo de processamento: {:3.0f} ms\nTempo de loop: {:3.0f} ms".format(self._processing_time*1000, self._loop_time*1000))
    
    def set_state(self, stateName):
        if stateName == "mainMenu":
            self._state.request_state_change(states.config_vision.ConfigVision(self))
        elif stateName == "gameLoop":
            self._state.request_state_change(states.game_loop.UiGameLoop(self))
        elif stateName == "configStrategy":
            self._state.request_state_change(states.config_strategy.ConfigStrategy(self))
        elif stateName == "configWorld":
            self._state.request_state_change(states.config_world.ConfigWorld(self))
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
        self.thread.join()
    
    def exit(self, signum, frame):
        self._state.request_quit()
    
    def run(self):
        self.thread = Thread(target=self.__loop__)
        self.thread.start()

    def runBlocking(self):
        self.__loop__()
        
    def addEvent(self, method, *args):
        self._events.put({"method": method, "args": args})
    
    def runQueuedEvents(self):
        while not self._events.empty():
            try:
                event = self._events.get_nowait()
                event["method"](*event["args"])
            except:
                print("Failed to run queued event")
        
    def __loop__(self):
        while True:
            loop_time = time.time()
            
            self.runQueuedEvents()
            
            try:
                self._state.update()
            except Exception as e:
                gui.mainWindow.MainWindow().logErrorMessage(traceback.format_exc())
                time.sleep(0.03)
                
            if self._state.QuitRequested:
                self._radioComm.sendZero()
                rh.RosHandler().terminateAll()
                break
            if self._state.StateChangeRequested:
                self._state = self._state.next_state()
                
            loop_time = time.time()-loop_time
            self.update_stats(0, loop_time)
            #time.sleep(1)
        
        
