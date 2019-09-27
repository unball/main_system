from gi.repository import Gtk
import gui.frameRenderer

from statics.static_classes import world
from vision.pixel2metric import meters2pixel, pixel2meters
from statics.world.game_elements.robot import Robot

import cv2
import numpy as np
import math

class parametrosEstrategia(gui.frameRenderer.frameRenderer):

    def __init__(self, vision):
        super().__init__(vision, "fr_strategy_notebook")
        self.frameShape = (350,471)
        
    def draw_rectangle(self, frame, position, size, angle, color=(0,255,0)):
        rect = (position, size, -angle*180/np.pi)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame,[box],0,color,2)
        cv2.circle(frame, (int(position[0]+size[0]/2*math.cos(-angle)), int(position[1]+size[0]/2*math.sin(-angle))), 3, (255,255,255), -1)
    
    def transformFrame(self, frame, originalFrame):
        height, width = self.frameShape
        frame = np.zeros((height,width,3), np.uint8)
        
        ellipseCenter = meters2pixel((0.75*world.fieldSide, 0), self.frameShape)
        ellipseAxis = (int(0.4*width/1.6),int(0.2*height/1.3))
        cv2.ellipse(frame, ellipseCenter, ellipseAxis, 0, 0, 360, (100,100,100), 1)
        blkballline,_ = meters2pixel((0.55*world.fieldSide,0), (height,width))
        cv2.line(frame, (blkballline,0), (blkballline,height), (100,100,100), 1)
        
        for robot in world.robots:
            if robot.pos[0] is not None and robot.pos[1] is not None:
                position = meters2pixel(robot.pos, (height,width))
                
                self.draw_rectangle(frame, position, (25,25), robot.th, color=(0,255,0))
                
                if robot.entity is not None:
                    cv2.putText(frame, str(robot.entity)[0], (int(position[0])-7, int(position[1])+5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255))
                
                robot.discretize(0.01)
                if len(robot.trajectory) > 0:
                    cv2.polylines(frame,[np.array([meters2pixel(x, (height,width)) for x in robot.trajectory[0]])],False,(255,255,255),1)
        
        bola = world.ball
        ballpos = meters2pixel(bola.pos, (height,width))
        cv2.circle(frame, ballpos, 5, (255,0,0), -1)
        
        
        return frame

    def create_ui_label(self):
        return Gtk.Label("Parâmetros da estratégia")
    
    def update_turning_radius(self, widget):
        self.parent.setTurningRadius(widget.get_value())
    
    def set_dynamic_possession(self, widget, value):
        self.parent.setDynamicPossession(value)
    
    def create_ui_content(self):
        builder = Gtk.Builder.new_from_file("strategy/parametrosEstrategia.ui")
        builder.get_object("min_dubins_radius_spin").connect("value-changed", self.update_turning_radius)
        builder.get_object("dynamic_possession").connect("state-set", self.set_dynamic_possession)
        
        return builder.get_object("main")

