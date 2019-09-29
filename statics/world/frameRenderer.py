from gi.repository import Gtk, Gdk
import gui.frameRenderer

from statics.static_classes import world
from vision.pixel2metric import meters2pixel, pixel2meters
from statics.world.game_elements.robot import Robot

import cv2
import numpy as np
import math

from pkg_resources import resource_filename

class elementsPositioner(gui.frameRenderer.frameRenderer):

    def __init__(self, vision):
        super().__init__(vision, "fr_world_notebook")
        self.__selectedRobot = 0
        self.__mousePosition = (0,0)
        self.__selector = "None"
        self.frameShape = (350,471)
        self.__movingRobot = None
        
        self.__robots = []
        self.__ball = (0,0)
        
    def draw_rectangle(self, frame, position, size, angle, color=(0,255,0)):
        rect = (position, size, -angle*180/np.pi)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame,[box],0,color,2)
        cv2.circle(frame, (int(position[0]+size[0]/2*math.cos(-angle)), int(position[1]+size[0]/2*math.sin(-angle))), 3, (255,255,255), -1)
        
    def cursorDistance(self, position):
        return abs(self.__mousePosition[0]-position[0])+abs(self.__mousePosition[1]-position[1])
    
    def transformFrame(self, frame, originalFrame):
        height, width = self.frameShape
        frame = np.zeros((height,width,3), np.uint8)
        
        if self.__selector == "Robot":
            self.draw_rectangle(frame, self.__mousePosition, (25,25), 0, color=(0,200,0))
        elif self.__selector == "Ball":
            cv2.circle(frame, self.__mousePosition, 5, (255,0,0), -1)
        
        ellipseCenter = meters2pixel((0.75*world.fieldSide, 0), self.frameShape)
        ellipseAxis = (int(0.4*width/1.6),int(0.2*height/1.3))
        cv2.ellipse(frame, ellipseCenter, ellipseAxis, 0, 0, 360, (100,100,100), 1)
        blkballline,_ = meters2pixel((0.55*world.fieldSide,0), (height,width))
        cv2.line(frame, (blkballline,0), (blkballline,height), (100,100,100), 1)
        
        for robot in world.robots:
            if robot.pos[0] is not None and robot.pos[1] is not None:
                position = meters2pixel(robot.pos, (height,width))
                
                if(self.__selector == "MoveRobot" and self.cursorDistance(position) < 20):
                    self.draw_rectangle(frame, position, (25,25), robot.th, color=(0,0,255))
                elif(self.__selector == "RemoveRobot" and self.cursorDistance(position) < 20):
                    self.draw_rectangle(frame, position, (25,25), robot.th, color=(255,0,0))
                else:
                    self.draw_rectangle(frame, position, (25,25), robot.th, color=(0,255,0))
                
                if robot.entity is not None:
                    cv2.putText(frame, str(robot.entity)[0], (int(position[0])-7, int(position[1])+5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255))
                
                robot.discretize(0.01)
                if len(robot.trajectory) > 0:
                    #print(np.array([meters2pixel(x, (height,width)) for x in robot.trajectory[0]]))
                    cv2.polylines(frame,[np.array([meters2pixel(x, (height,width)) for x in robot.trajectory[0]])],False,(255,255,255),1)
        
        bola = world.ball
        ballpos = meters2pixel(bola.pos, (height,width))
        cv2.circle(frame, ballpos, 5, (255,0,0), -1)
        cv2.arrowedLine(frame, ballpos, (ballpos[0]+int(bola.vel[0]*50), ballpos[1]-int(bola.vel[1]*50)), (255,0,0), 1)
        
        
        return frame

    def create_ui_label(self):
        return Gtk.Label("Posicionador de elementos")
        
    def findNearRobot(self):
        nearRobots = [r for r in world.robots if self.cursorDistance(meters2pixel(r.pos, self.frameShape)) < 20]
        if(len(nearRobots) != 0):
            return nearRobots[0]
        else: return None
        
    def frameMouseOver(self, widget, event):
        self.__mousePosition = (int(event.x), int(event.y))
        if self.__selector == "MoveRobot" and self.__movingRobot is not None:
            position = pixel2meters((int(event.x), int(event.y)), self.frameShape)
            self.__movingRobot.update(position[0], position[1], self.__movingRobot.th)
    
    def frameClick(self, widget, event):
        if world.manualMode == False: return
        
        if self.__selector == "Robot":
            newRobot = Robot()
            position = pixel2meters((int(event.x), int(event.y)), self.frameShape)
            newRobot.update(position[0], position[1])
            world.robots.append(newRobot)
        elif self.__selector == "Ball":
            position = pixel2meters((int(event.x), int(event.y)), self.frameShape)
            world.ball.update(position[0], position[1])
            world.ball.vel = (-1,-1)
        elif self.__selector == "MoveRobot":
            nearRobot = self.findNearRobot()
            if nearRobot is not None:
                self.__movingRobot = nearRobot
        elif self.__selector == "RemoveRobot":
            for idx,robot in enumerate(world.robots):
                if self.cursorDistance(meters2pixel(robot.pos, self.frameShape)) < 20:
                    del world.robots[idx]
    
    def frameScroll(self, widget, event):
        if self.__selector == "MoveRobot":
            nearRobot = self.findNearRobot()
            if nearRobot is not None:
                nearRobot.th = nearRobot.th+event.delta_y*0.1
            
    def frameRelease(self, widget, event):
        self.__movingRobot = None
        
    def setSelector(self, widget, selector):
        value = widget.get_active()
        
        if value:
            self.__selector = selector
            
    def setManualMode(self, widget, value):
        world.setManualMode(value)
    
    def create_ui_content(self):
        builder = Gtk.Builder.new_from_file(resource_filename(__name__, "elementPositioner.ui"))
        
        builder.get_object("manual_mode").connect("state-set", self.setManualMode)
        builder.get_object("adicionar_aliado").connect("toggled", self.setSelector, "Robot")
        builder.get_object("adicionar_bola").connect("toggled", self.setSelector, "Ball")
        builder.get_object("mover_aliado").connect("toggled", self.setSelector, "MoveRobot")
        builder.get_object("remover_aliado").connect("toggled", self.setSelector, "RemoveRobot")
        
        gui.mainWindow.MainWindow().getObject("world_frame_event").connect("motion-notify-event", self.frameMouseOver)
        gui.mainWindow.MainWindow().getObject("world_frame_event").connect("button-press-event", self.frameClick)
        gui.mainWindow.MainWindow().getObject("world_frame_event").connect("button-release-event", self.frameRelease)
        gui.mainWindow.MainWindow().getObject("world_frame_event").add_events(Gdk.EventMask.SMOOTH_SCROLL_MASK)
        gui.mainWindow.MainWindow().getObject("world_frame_event").connect("scroll-event", self.frameScroll)
        
        return builder.get_object("main")


