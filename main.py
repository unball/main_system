#!/usr/bin/env python3

from statics import static_classes
from strategy.movimentsDecider import Attacker
from strategy.movimentsDecider import MovimentsDecider

import gi
gi.require_version('Gtk', '3.0')

import gui.mainWindow
import cv2

def main():
    print("Usando opencv " + cv2.__version__)
    
    # Inicia a GUI
    w = gui.mainWindow.MainWindow()
    w.run()

if __name__ == "__main__":
    main()
    r = static_classes.world.robots[0]
    ent = Attacker()
    path = 0
    ent.possess(path, 0)
    r.entity.path = MovimentsDecider().shortestTragectory((0,0,0),(1,1,0), 0.01)
