#!/usr/bin/env python3
from threading import Thread,Timer
from states.main_menu import MainMenu
from states.game_loop import GameLoop

import gi
gi.require_version('Gtk', '3.0')

import gui.mainWindow
import cv2
import time

state = None

def start():
    global state
    state = GameLoop()
    while True:
        state.update()
        if state.QuitRequested:
            break
        if state.StateChangeRequested:
            state = state.next_state()
        #time.sleep(1)

def main():
    print("Usando opencv " + cv2.__version__)
    
    # Inicia a thread do sistema
    gameThread = Thread(target=start)
    gameThread.start()
    
    # Inicia a GUI
    w = gui.mainWindow.MainWindow()
    w.run()

    if state: state.request_quit()

if __name__ == "__main__":
    main()
