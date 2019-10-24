from gi.repository import Gtk, Gdk
import gui.frameRenderer

from gui.drawing import Drawing
from statics.static_classes import world
from vision.pixel2metric import meters2pixel, pixel2meters
from statics.world.game_elements.robot import Robot

import cv2
import numpy as np
import math

from pkg_resources import resource_filename


def draw_rectangle(frame, position, size, angle, color=(0,255,0)):
    rect = (position, size, -angle*180/np.pi)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(frame,[box],0,color,2)
    cv2.circle(frame, (int(position[0]+size[0]/2*math.cos(-angle)), int(position[1]+size[0]/2*math.sin(-angle))), 3, (255,255,255), -1)

def strategyFrame(frameShape, step=0.01):
    height, width = frameShape
    frame = np.zeros((height,width,3), np.uint8)
    
    Drawing.draw_field(frame)

    # Desenha area interna do campo
    Drawing.draw_internal_field(frame, frameShape)

    ellipseCenter = meters2pixel((0.75*world.fieldSide, 0), frameShape)
    ellipseAxis = (int(0.3*width/1.6),int(0.4*height/1.3))
    cv2.ellipse(frame, ellipseCenter, ellipseAxis, 0, 0, 360, (100,100,100), 1)
    blkballline,_ = meters2pixel((0.55*world.fieldSide,0), (height,width))
    cv2.line(frame, (blkballline,0), (blkballline,height), (100,100,100), 1)
    
    for robot in world.robots[:3]:
        if robot.pos[0] is not None and robot.pos[1] is not None:
            position = meters2pixel(robot.pos, (height,width))
            
            draw_rectangle(frame, position, (25,25), robot.th, color=(0,255,0))
            
            if robot.entity is not None:
                cv2.putText(frame, str(robot.entity)[0], (int(position[0])-7, int(position[1])+5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255))
            
            robot.discretize(step)
            if len(robot.trajectory) > 0:
                cv2.polylines(frame,[np.array([meters2pixel(x, (height,width)) for x in robot.trajectory[0]])],False,(255,255,255),1)
    
    bola = world.ball
    ballpos = meters2pixel(bola.pos, (height,width))
    cv2.circle(frame, ballpos, 5, (255,0,0), -1)
    cv2.arrowedLine(frame, ballpos, (ballpos[0]+int(bola.vel[0]*50), ballpos[1]-int(bola.vel[1]*50)), (255,0,0), 1)
    
    
    return frame

class parametrosEstrategia(gui.frameRenderer.frameRenderer):

    def __init__(self, vision):
        super().__init__(vision, "fr_strategy_notebook")
        self.frameShape = (350,471)

    def create_ui_label(self):
        return Gtk.Label("Parâmetros da estratégia")
    
    def update_turning_radius(self, widget):
        self.gameThread.addEvent(self.parent.setTurningRadius, widget.get_value())
    
    def update_spin(self, widget):
        print("EXECUTEI")
        self.gameThread.addEvent(self.parent.setStep, widget.get_value())
    
    def set_dynamic_possession(self, widget, value):
        self.gameThread.addEvent(self.parent.setDynamicPossession, value)

    def transformFrame(self, frame, originalFrame):
        processed_image = strategyFrame(self.frameShape, step=self.parent.step)
        return cv2.cvtColor(processed_image, cv2.COLOR_RGB2BGR)
    
    def reset_radius(self, widget, spinButton):
         spinButton.set_value(0.053)
    
    def reset_spin(self, widget, stepButton):
         stepButton.set_value(0.005)
        
    def strategy_setPosition(self, widget, event):
        if not self.isFrameRendererSelected(): return
        self.gameThread.addEvent(world.setMainPoint, (int(event.x), int(event.y)))

    def create_ui_content(self):
        builder = Gtk.Builder.new_from_file(resource_filename(__name__, "parametrosEstrategia.ui"))

        spinButton = builder.get_object("min_dubins_radius_spin")
        spinButton.connect("value-changed", self.update_turning_radius)
        spinButton.set_value(self.parent.decider.turning_radius)

        stepButton = builder.get_object("step_spin")
        stepButton.connect("value-changed", self.update_spin)
        stepButton.set_value(self.parent.step)

        builder.get_object("min_dubins_radius_spin_reset").connect("clicked", self.reset_radius, spinButton)
        builder.get_object("step_spin_reset").connect("clicked", self.reset_spin, stepButton)
        builder.get_object("dynamic_possession").connect("state-set", self.set_dynamic_possession)


        gui.mainWindow.MainWindow().getObject("strategy_frame_event").connect("button-press-event", self.strategy_setPosition)        
        
        return builder.get_object("main")


class gerenciadorEntidades(gui.frameRenderer.frameRenderer):

    def __init__(self, vision):
        super().__init__(vision, "fr_strategy_notebook")
        self.frameShape = (350,471)

    def create_ui_label(self):
        return Gtk.Label("Gerenciador de entidades")

    def transformFrame(self, frame, originalFrame):
        return strategyFrame(self.frameShape, step=self.parent.step)
    
    def create_ui_content(self):
        builder = Gtk.Builder.new_from_file(resource_filename(__name__, "gerenciadorEntidades.ui"))
        
        return builder.get_object("main")


class elementsPositioner(gui.frameRenderer.frameRenderer):

    def __init__(self, vision):
        super().__init__(vision, "fr_strategy_notebook")
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

        Drawing.draw_internal_field(frame, self.frameShape)
        
        if self.__selector == "Robot":
            self.draw_rectangle(frame, self.__mousePosition, (25,25), 0, color=(0,200,0))
        elif self.__selector == "Ball":
            cv2.circle(frame, self.__mousePosition, 5, (255,0,0), -1)
        
        ellipseCenter = meters2pixel((0.75*world.fieldSide, 0), self.frameShape)
        ellipseAxis = (int(0.3*width/1.6),int(0.4*height/1.3))
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
                
                robot.discretize(gui.mainWindow.MainWindow().gameThread.strategySystem.step)
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
        if not self.isFrameRendererSelected(): return
        self.__mousePosition = (int(event.x), int(event.y))
        if self.__selector == "MoveRobot" and self.__movingRobot is not None:
            position = pixel2meters((int(event.x), int(event.y)), self.frameShape)
            self.__movingRobot.update(position[0], position[1], self.__movingRobot.th)
    
    def frameClick(self, widget, event):
        if not self.isFrameRendererSelected(): return
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
        if not self.isFrameRendererSelected(): return
        if self.__selector == "MoveRobot":
            nearRobot = self.findNearRobot()
            if nearRobot is not None:
                nearRobot.th = nearRobot.th+event.delta_y*0.1
            
    def frameRelease(self, widget, event):
        if not self.isFrameRendererSelected(): return
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
        
        gui.mainWindow.MainWindow().getObject("strategy_frame_event").connect("motion-notify-event", self.frameMouseOver)
        gui.mainWindow.MainWindow().getObject("strategy_frame_event").connect("button-press-event", self.frameClick)
        gui.mainWindow.MainWindow().getObject("strategy_frame_event").connect("button-release-event", self.frameRelease)
        gui.mainWindow.MainWindow().getObject("strategy_frame_event").add_events(Gdk.EventMask.SMOOTH_SCROLL_MASK)
        gui.mainWindow.MainWindow().getObject("strategy_frame_event").connect("scroll-event", self.frameScroll)
        
        return builder.get_object("main")


