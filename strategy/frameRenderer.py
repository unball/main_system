from gi.repository import Gtk
import gui.frameRenderer

from statics.static_classes import world
from vision.pixel2metric import meters2pixel, pixel2meters
from statics.world.game_elements.robot import Robot

import cv2
import numpy as np

class movementSimulator(gui.frameRenderer.frameRenderer):

    def __init__(self, vision):
        super().__init__(vision, "fr_strategy_notebook")
        self.__selectedRobot = 0
        self.__mousePosition = (0,0)
        self.__selector = "None"
        self.frameShape = (520,640)
        
        self.__robots = []
        self.__ball = (0,0)
        
    def draw_rectangle(self, frame, position, size, angle, color=(0,255,0)):
        rect = (position, size, angle)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame,[box],0,color,2)
    
    def transformFrame(self, frame, originalFrame):
        height, width = self.frameShape
        frame = np.zeros((height,width,3), np.uint8)
        
        if self.__selector == "Robot":
            self.draw_rectangle(frame, self.__mousePosition, (25,25), 0, color=(0,200,0))
        elif self.__selector == "Ball":
            cv2.circle(frame, self.__mousePosition, 5, (255,0,0), -1)
        
        for robot in self.__robots:
            if robot.pos[0] is not None and robot.pos[1] is not None:
                position = meters2pixel(robot.pos, (height,width))
                
                if self.__selector == "Entity" and (abs(position[0]-self.__mousePosition[0]) + abs(position[1]-self.__mousePosition[1])) < 25:
                    self.draw_rectangle(frame, position, (25,25), robot.th, color=(0,0,255))
                else:
                    self.draw_rectangle(frame, position, (25,25), robot.th, color=(0,255,0))
                
                if robot.entity is not None:
                    cv2.putText(frame, str(robot.entity)[0], (int(position[0])-10, int(position[1])+10), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,255,255))
                
                robot.discretize(0.01)
                if len(robot.trajectory) > 0:
                    #print(np.array([meters2pixel(x, (height,width)) for x in robot.trajectory[0]]))
                    cv2.polylines(frame,[np.array([meters2pixel(x, (height,width)) for x in robot.trajectory[0]])],False,(255,255,255),1)
        
        ballpos = meters2pixel(self.__ball, (height,width))
        cv2.circle(frame, ballpos, 5, (255,0,0), -1)
        
        
        return frame

    def create_ui_label(self):
        return Gtk.Label("Simulador de movimentos")
        
    def frameMouseOver(self, widget, event):
        self.__mousePosition = (int(event.x), int(event.y))
    
    def frameClick(self, widget, event):
        if self.__selector == "Robot":
            newRobot = Robot()
            position = pixel2meters((int(event.x), int(event.y)), self.frameShape)
            newRobot.update(position[0], position[1])
            self.__robots.append(newRobot)
        elif self.__selector == "Ball":
            self.__ball = pixel2meters((int(event.x), int(event.y)), self.frameShape)
        elif self.__selector == "Entity":
#            for robot in self.__robots:
#                if robot.pos[0] is not None and robot.pos[1] is not None:
#                    position = meters2pixel(robot.pos, self.frameShape)
#                    if (abs(position[0]-self.__mousePosition[0]) + abs(position[1]-self.__mousePosition[1])) < 25:
#                   
            pass
        
    def setSelector(self, widget, selector):
        value = widget.get_active()
        
        if value:
            self.__selector = selector
    
    def create_ui_content(self):
        builder = Gtk.Builder.new_from_file("strategy/movementSimulator.ui")
        
        self.__selector_addAlly = builder.get_object("adicionar_aliado")
        self.__selector_addAlly.connect("toggled", self.setSelector, "Robot")
        
        self.__selector_addBall = builder.get_object("adicionar_bola")
        self.__selector_addBall.connect("toggled", self.setSelector, "Ball")
        
        self.__selector_possuir = builder.get_object("possuir")
        self.__selector_possuir.connect("toggled", self.setSelector, "Entity")
        
        gui.mainWindow.MainWindow().getObject("strategy_frame_event").connect("motion-notify-event", self.frameMouseOver)
        gui.mainWindow.MainWindow().getObject("strategy_frame_event").connect("button-press-event", self.frameClick)
        
        return builder.get_object("main")


class parametrosEstrategia(gui.frameRenderer.frameRenderer):

    def __init__(self, vision):
        super().__init__(vision, "fr_strategy_notebook")
    
    def transformFrame(self, frame, originalFrame):
        height, width = (520,640)
        frame = np.zeros((height,width,3), np.uint8)
        
        delta_p,_ = meters2pixel((self.parent.decider.delta,0), (height,width))
        delta_n,_ = meters2pixel((-self.parent.decider.delta,0), (height,width))
        cv2.line(frame, (delta_p,0), (delta_p,height), (150,150,150), 1)
        cv2.line(frame, (delta_n,0), (delta_n,height), (150,150,150), 1)
        
        for robot in world.robots:
            if robot.pos[0] is not None and robot.pos[1] is not None:
                position = meters2pixel(robot.pos, (height,width))
                rect = (position,(25,25),robot.th)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame,[box],0,(0,255,0),2)
                if robot.entity is not None:
                    cv2.putText(frame, str(robot.entity)[0], (int(position[0])-10, int(position[1])+10), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,255,255))
                
                robot.discretize(0.01)
                if len(robot.trajectory) > 0:
                    #print(np.array([meters2pixel(x, (height,width)) for x in robot.trajectory[0]]))
                    cv2.polylines(frame,[np.array([meters2pixel(x, (height,width)) for x in robot.trajectory[0]])],False,(255,255,255),1)
        
        bola = world.ball
        ballpos = meters2pixel(bola.pos, (height,width))
        cv2.circle(frame, ballpos, 5, (255,0,0), -1)
        
        return frame

    def create_ui_label(self):
        return Gtk.Label("Parâmetros da estratégia")
    
    def update_turning_radius(self, widget):
        self.parent.setTurningRadius(widget.get_value())
    
    def create_ui_content(self):
        builder = Gtk.Builder.new_from_file("strategy/parametrosEstrategia.ui")
        builder.get_object("min_dubins_radius_spin").connect("value-changed", self.update_turning_radius)
        
        return builder.get_object("main")

