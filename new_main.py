#!/usr/bin/env python3
from threading import Thread,Timer
from states.main_menu import MainMenu
from states.game_loop import GameLoop

import gi
gi.require_version('Gtk', '3.0')

import gui.mainWindow
import cv2
import time

def main():
    print("Usando opencv " + cv2.__version__)
    
    # Inicia a GUI
    w = gui.mainWindow.MainWindow()
    w.run()

if __name__ == "__main__":
    main()
