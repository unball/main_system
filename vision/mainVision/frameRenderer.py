from gi.repository import Gtk, GLib
import statics.configFile
import numpy as np
import gui.mainWindow
import cv2
import gui.singleton
import time
import vision.mainVision
import gui.frameRenderer

class clickedPoints():
	def __init__(self, configFileVariableName, maxSize):
		self.__points = statics.configFile.getValue(configFileVariableName, [])
		self.__max_size = maxSize
		self.__config_file_variable_name = configFileVariableName
	
	def append(self, point):
		if len(self.__points) >= self.__max_size:
			self.__points.clear()
			
		if len(self.__points) < self.__max_size:
			self.__points.append(point)
			
		if len(self.__points) == self.__max_size:
			statics.configFile.setValue(self.__config_file_variable_name, self.__points)
	
	def isFilled(self):
		return len(self.__points) == self.__max_size
		
	def length(self):
		return len(self.__points)
	
	def getSorted(self):
		return sorted(self.__points.copy(), key=sum)
	
	def getPointPixels(self, index, shape):
		return (round(self.__points[index][0]*shape[0]), round(self.__points[index][1]*shape[1]))

class cortarCampo(gui.frameRenderer.frameRenderer):
	
	def __init__(self, vision):
		super().__init__(vision)
		# Variables
		self.__pointer_position = None
		self.__show_warpped = False
		self.__frame_shape = None
		
		self.__points = statics.configFile.getValue("points", [])
		self.__crop_points = statics.configFile.getValue("cortarCampo_crop_points", [])
		
	def sortPoints(self,points):
		if len(points) == 4:
			points.sort(key=sum)
			if points[1][0] > points[2][0]:
				tmp = points[1]
				points[1] = points[2]
				points[2] = tmp
		return points
	
	def update_points(self, point):
		if self.__show_warpped: return
		
		if self.parentVision.use_homography:
			if len(self.__points) >= 4:
				self.__points.clear()
				
			if len(self.__points) < 4:
				self.__points.append([point[0]/self.__frame_shape[0], point[1]/self.__frame_shape[1]])
				
			if len(self.__points) == 4 and self.__frame_shape is not None:
				statics.configFile.setValue("points", self.__points)
				
				self.parentVision.updateHomography(self.sortPoints(self.__points.copy()), self.__frame_shape)
		else:
			if len(self.__crop_points) >= 2:
				self.__crop_points.clear()
				
			if len(self.__crop_points) < 2:
				self.__crop_points.append([point[0]/self.__frame_shape[0], point[1]/self.__frame_shape[1]])
				
			if len(self.__crop_points) == 2 and self.__frame_shape is not None:
				statics.configFile.setValue("cortarCampo_crop_points", self.__crop_points)
				
				self.parentVision.updateCropPoints(self.__crop_points.copy())
			
	
	def set_pointer_position(self, position):
		self.__pointer_position = position
	
	def set_show_mode(self, widget, value):
		self.__show_warpped = value
	
	def set_crop_mode(self, widget, value):
		self.parentVision.setUseHomography(value)
	
	def get_point_pixels(self, index):
		return (round(self.__points[index][0]*self.__frame_shape[0]), round(self.__points[index][1]*self.__frame_shape[1]))
		#return (self.__points[index][0], self.__points[index][1])
		
	def get_crop_point_pixels(self, index):
		return (round(self.__crop_points[index][0]*self.__frame_shape[0]), round(self.__crop_points[index][1]*self.__frame_shape[1]))
		
	def transformFrame(self, frame, originalFrame):
		self.__frame_shape = frame.shape
		
		if(self.__show_warpped):
			return cv2.cvtColor(self.parentVision.warp(frame), cv2.COLOR_RGB2BGR)
		
		if not self.parentVision.use_homography:
			if len(self.__crop_points) == 1:
				p0 = self.get_crop_point_pixels(0)
				cv2.circle(frame, p0, 5, (255,255,255), thickness=-1)
				cv2.rectangle(frame, p0, (self.__pointer_position[0], self.__pointer_position[1]), (255,255,255))
			elif len(self.__crop_points) == 2:
				p0 = self.get_crop_point_pixels(0)
				p1 = self.get_crop_point_pixels(1)
				cv2.circle(frame, p0, 5, (0,255,0), thickness=-1)
				cv2.circle(frame, p1, 5, (0,255,0), thickness=-1)
				cv2.rectangle(frame, p0, p1, (0,255,0), thickness=2)
			return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		
		color = (0,255,0) if len(self.__points) == 4 else (255,255,255)
		
		# Draw line for each pair of points
		if len(self.__points) > 1:
			for i in range(len(self.__points)-1):
				cv2.line(frame, self.get_point_pixels(i), self.get_point_pixels(i+1), color, thickness=2)
		
		# Closes rectangle
		if len(self.__points) == 4:
			cv2.line(frame, self.get_point_pixels(0), self.get_point_pixels(3), color, thickness=2)
		
		# Draw helping line from last chosen point to current mouse position
		if self.__pointer_position and len(self.__points) > 0 and len(self.__points) < 4:
			cv2.line(frame, self.get_point_pixels(-1), (self.__pointer_position[0], self.__pointer_position[1]), (255,255,255))
			# Draw extra line from first chosen point to current position when it's the last point to be chosen
			if len(self.__points) == 3:
				cv2.line(frame, self.get_point_pixels(0), (self.__pointer_position[0], self.__pointer_position[1]), color)
		
		for i in range(len(self.__points)):
			cv2.circle(frame, self.get_point_pixels(i), 5, color, thickness=-1)
		
		return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		
	def cortarCampo_update_points(self, widget, event):
		if gui.mainWindow.MainWindow().selectedFrameRenderer == self:
			self.update_points([int(event.x), int(event.y)])
	
	def cortarCampo_mouseOver(self, widget, event):
		if gui.mainWindow.MainWindow().selectedFrameRenderer == self:
			self.set_pointer_position((int(event.x), int(event.y)))

	def create_ui_label(self):
		return Gtk.Label("Cortar Campo")
	
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file("vision/mainVision/cortarCampo.ui")
		builder.get_object("campo_switch").connect("state-set", self.set_show_mode)
		builder.get_object("homografia_switch").connect("state-set", self.set_crop_mode)
		builder.get_object("homografia_switch").set_state(self.parentVision.use_homography)
		gui.mainWindow.MainWindow().getObject("frame_event").connect("button-press-event", self.cortarCampo_update_points)
		gui.mainWindow.MainWindow().getObject("frame_event").connect("motion-notify-event", self.cortarCampo_mouseOver)

		return builder.get_object("main")
		

class segmentarPreto(gui.frameRenderer.frameRenderer):

	def __init__(self, vision):
		super().__init__(vision)
		# Variables
		self.__ui_elements = ["fundo_hmin", "fundo_smin", "fundo_vmin", "fundo_hmax", "fundo_smax", "fundo_vmax"]
		
	def update_hsv_interval(self, widget, index):
		self.parentVision.atualizarPretoHSV(int(widget.get_value()), index)
	
	def transformFrame(self, frame, originalFrame):
		img_warped = self.parentVision.warp(frame)
		return cv2.cvtColor(self.parentVision.segmentarFundo(img_warped), cv2.COLOR_GRAY2RGB)

	def create_ui_label(self):
		return Gtk.Label("Segmentar Preto")
	
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file("vision/mainVision/segmentarPreto.ui")
		for index,name in enumerate(self.__ui_elements):
			element = builder.get_object(name)
			element.set_value(self.parentVision.preto_hsv[index])
			element.connect("value-changed", self.update_hsv_interval, index)
		
		return builder.get_object("main")

class segmentarTime(gui.frameRenderer.frameRenderer):
	def __init__(self, vision):
		super().__init__(vision)
		# Variables
		self.__ui_elements = ["time_hmin", "time_smin", "time_vmin", "time_hmax", "time_smax", "time_vmax"]

	def update_hsv_interval(self, widget, index):
		self.parentVision.atualizarTimeHSV(int(widget.get_value()), index)
	
	def transformFrame(self, frame, originalFrame):
		img_warped = self.parentVision.warp(frame)
		return cv2.cvtColor(self.parentVision.segmentarTime(img_warped), cv2.COLOR_GRAY2RGB)

	def create_ui_label(self):
		return Gtk.Label("Segmentar Time")
	
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file("vision/mainVision/segmentarTime.ui")
		for index,name in enumerate(self.__ui_elements):
			element = builder.get_object(name)
			element.set_value(self.parentVision.time_hsv[index])
			element.connect("value-changed", self.update_hsv_interval, index)
		
		return builder.get_object("main")

class segmentarBola(gui.frameRenderer.frameRenderer):
	def __init__(self, vision):
		super().__init__(vision)
		# Variables
		self.__ui_elements = ["bola_hmin", "bola_smin", "bola_vmin", "bola_hmax", "bola_smax", "bola_vmax"]
	
	def update_hsv_interval(self, widget, index):
		self.parentVision.atualizarBolaHSV(int(widget.get_value()), index)
	
	def transformFrame(self, frame, originalFrame):
		img_warped = self.parentVision.warp(frame)
		return cv2.cvtColor(self.parentVision.segmentarBola(img_warped), cv2.COLOR_GRAY2RGB)

	def create_ui_label(self):
		return Gtk.Label("Segmentar Bola")
		
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file("vision/mainVision/segmentarBola.ui")
		for index,name in enumerate(self.__ui_elements):
			element = builder.get_object(name)
			element.set_value(self.parentVision.bola_hsv[index])
			element.connect("value-changed", self.update_hsv_interval, index)
		
		return builder.get_object("main")

class identificarRobos(gui.frameRenderer.frameRenderer):
	def __init__(self, vision):
		super().__init__(vision)
	
	def updateRobotsInfo(self, robos, bola):
		for robo in robos:
			if robo.ui:
				robo.ui["idLabel"].set_text("{0}".format(robo.identificador))
				robo.ui["posicaoLabel"].set_text("Posição: x: {:.2f} m, y: {:.2f} m".format(robo.centro[0], robo.centro[1]))
				robo.ui["anguloLabel"].set_text("Ângulo {:.1f}º".format(robo.angulo))
				robo.ui["estadoLabel"].set_text("Estado: " + robo.estado)
			else:
				flowBoxChild = Gtk.FlowBoxChild()
				Gtk.StyleContext.add_class(flowBoxChild.get_style_context(), "roboRow")
				columnBox = Gtk.Box()
				idBox = Gtk.Box()
				idBox.set_orientation(Gtk.Orientation.VERTICAL)
				idBox.set_margin_left(10)
				idBox.set_margin_right(10)
				idBox.set_margin_top(10)
				idBox.set_margin_bottom(10)
				roboLabel = Gtk.Label("Robô")
				idLabel = Gtk.Label("{0}".format(robo.identificador))
				Gtk.StyleContext.add_class(idLabel.get_style_context(), "roboId")
				idLabel.set_size_request(80,-1)
				idBox.add(roboLabel)
				idBox.add(idLabel)
				infoBox = Gtk.Box()
				estadoLabel = Gtk.Label("Estado: " + robo.estado)
				estadoLabel.set_halign(Gtk.Align.START)
				posicaoLabel = Gtk.Label("Posição: (x: {:.2f}, y: {:.2f})".format(robo.centro[0], robo.centro[1]))
				posicaoLabel.set_halign(Gtk.Align.START)
				anguloLabel = Gtk.Label("Ângulo {:.1f}º".format(robo.angulo))
				anguloLabel.set_halign(Gtk.Align.START)
				infoBox.set_orientation(Gtk.Orientation.VERTICAL)
				infoBox.set_valign(Gtk.Align.CENTER)
				infoBox.add(estadoLabel)
				infoBox.add(posicaoLabel)
				infoBox.add(anguloLabel)
				columnBox.add(idBox)
				columnBox.add(infoBox)
				flowBoxChild.add(columnBox)
				self.__timeFlow.add(flowBoxChild)
				robo.ui = {"idLabel": idLabel, "posicaoLabel": posicaoLabel, "anguloLabel": anguloLabel, "estadoLabel": estadoLabel}
			self.__timeFlow.show_all()
			
		if bola is not None:
			self.__bolaEstado.set_text("Estado: Identificada")
			self.__bolaPosicao.set_text("Posição: x: {:.2f} m, y: {:.2f} m".format(bola[0][0], bola[0][1]))
		else:
			self.__bolaEstado.set_text("Estado: Não-Identificada")
			
	
	def transformFrame(self, frame, originalFrame):
		processed_frame = self.parentVision.ui_process(frame)
		
		robosAliados = self.parentVision.robosAliados
		bola = self.parentVision.bola
		
		GLib.idle_add(self.updateRobotsInfo, robosAliados, bola)
		
		return cv2.cvtColor(processed_frame, cv2.COLOR_RGB2BGR)

	def create_ui_label(self):
		return Gtk.Label("Visão em alto nível")
	
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file("vision/mainVision/identificarRobos.ui")
		self.__timeFlow = builder.get_object("time_flow")
		self.__bolaEstado = builder.get_object("bola_estado")
		self.__bolaPosicao = builder.get_object("bola_posicao")
		return builder.get_object("main")
