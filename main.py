#!/usr/bin/env python3

import gi
gi.require_version('Gtk', '3.0')

from statics import static_classes
from strategy.movimentsDecider import Attacker
from strategy.movimentsDecider import MovimentsDecider
import sys
import states.gameThread
from statics.static_classes import world
import roshandler.roshandler as rh
import signal
import states.game_loop



import gui.mainWindow
import cv2
#TODO: implementar execucao simples sem GUI (iniciar thread com estado de gameloop)

def main():
    print("Usando opencv " + cv2.__version__)
    
    # Inicia a GUI
    if len(sys.argv) > 1 and sys.argv[1] == "--no-gui":
        world.startGame()
        gameThread = states.gameThread.GameThread(initialState=states.game_loop.GameLoop)
        gameThread.runBlocking()
    else:
        w = gui.mainWindow.MainWindow()
        w.run()

if __name__ == "__main__":
    main()
