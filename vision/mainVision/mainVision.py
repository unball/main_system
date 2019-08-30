import cv2
import statics.configFile
import numpy as np
import vision.pixel2metric
import vision.mainVision.frameRenderer
from gi.repository import GLib
import vision.vision
import gui.mainWindow
from statics.static_classes import world
from statics import field

class RoboAdversario():
	def __init__(self, centro, angulo):
		self.centro = centro
		self.angulo = angulo
		
class RoboAliado():
	def __init__(self, identificador):
		self.identificador = identificador
		self.centro = (-1,-1)
		self.centroPixels = (-1,-1)
		self.angulo = -1
		self.estado = "Não-Identificado"
		self.ui = None

class MainVision(vision.vision.Vision):
	def __init__(self):
		self.__robosAliados = [
			RoboAliado(0),
			RoboAliado(1),
			RoboAliado(2),
			RoboAliado(3),
			RoboAliado(4)
		]
		self.__todosReconhecidos = False
		self.__robosAdversarios = []
		self.__bola = None
		self.__angles = np.array([0, 90, 180, -90, -180])
		self.__homography = None
		self.__default_preto_hsv = [0,94,163,360,360,360]
		self.__default_time_hsv = [13,0,0,32,360,360]
		self.__default_bola_hsv = [0, 117, 0, 98, 360, 360]
		self.__current_frame_shape = None
		super().__init__()

	def config_init(self):
		self.__preto_hsv = np.array(statics.configFile.getValue("preto_hsv_interval", self.__default_preto_hsv))
		self.__time_hsv = np.array(statics.configFile.getValue("time_hsv_interval", self.__default_time_hsv))
		self.__bola_hsv = np.array(statics.configFile.getValue("bola_hsv_interval", self.__default_bola_hsv))
		self.__homography = np.array(statics.configFile.getValue("homography_matrix"), None)
		self.__use_homography = statics.configFile.getValue("use_homography", True)
		self.__crop_points = statics.configFile.getValue("crop_points")
		self.__homography_points = statics.configFile.getValue("homography_points")


	def ui_init(self):
		self.frameRenderers = {
			"cortarCampo": vision.mainVision.frameRenderer.cortarCampo(self),
			"segmentarFundo": vision.mainVision.frameRenderer.segmentarPreto(self),
			"segmentarTime": vision.mainVision.frameRenderer.segmentarTime(self),
			"segmentarBola": vision.mainVision.frameRenderer.segmentarBola(self),
			"identificarRobos": vision.mainVision.frameRenderer.identificarRobos(self),
		}
		#self.selectedFrameRenderer = self.frameRenderers[0]
		GLib.idle_add(gui.mainWindow.MainWindow().set_frame_renderer, 0)
	
	@property
	def robosAliados(self):
		return self.__robosAliados
	
	@property
	def bola(self):
		return self.__bola
	
	@property
	def use_homography(self):
		return self.__use_homography
	
	@property
	def preto_hsv(self):
		return self.__preto_hsv
	
	@property
	def time_hsv(self):
		return self.__time_hsv
	
	@property
	def bola_hsv(self):
		return self.__bola_hsv
	
	def atualizarPretoHSV(self, value, index):
		self.__preto_hsv[index] = value
		config = statics.configFile.getConfig()
		config["preto_hsv_interval"][index] = value
		statics.configFile.saveConfig(config)
	
	def atualizarTimeHSV(self, value, index):
		self.__time_hsv[index] = value
		config = statics.configFile.getConfig()
		config["time_hsv_interval"][index] = value
		statics.configFile.saveConfig(config)
	
	def atualizarBolaHSV(self, value, index):
		self.__bola_hsv[index] = value
		config = statics.configFile.getConfig()
		config["bola_hsv_interval"][index] = value
		statics.configFile.saveConfig(config)
	
	def obterRobosAliados(self):
		return self.__robosAliados
		
	def atualizarRobos(self, robosAliados, robosAdversariosIdentificados, bola):
		# Computa novos robos inimigos
		self.__robosAdversarios = [RoboAdversario(robo[0], robo[1]) for robo in robosAdversariosIdentificados]
		
		# Atualiza posição da bola
		self.__bola = bola
		
		todosReconhecidos = True
		for i,r in enumerate(robosAliados):
			if r[3] == True:
				self.__robosAliados[i].centro = r[1]
				self.__robosAliados[i].centroPixels = r[4]
				self.__robosAliados[i].angulo = r[2]
				self.__robosAliados[i].estado = "Identificado"
			else:
				todosReconhecidos = False
				self.__robosAliados[i].estado = "Não-Identificado"
		self.__todosReconhecidos = todosReconhecidos
	
	def getHomography(self, shape):
		if self.__current_frame_shape != shape:
			self.updateHomography(self.__homography_points, shape)
		return self.__homography
	
	def updateHomography(self, points, shape):
		height, width, _ = shape
		self.__current_frame_shape = shape
		self.__homography_points = points
		key_points = np.array(points) * np.array([height, width])
		frame_points = np.array([[0,0],[0, height],[width,0],[width,height]])
		h, mask = cv2.findHomography(key_points, frame_points, cv2.RANSAC)
		statics.configFile.setValue("homography_matrix", h.tolist())
		statics.configFile.setValue("homography_points", points)
		
		self.__homography = h
	
	def updateCropPoints(self, points):
		self.__crop_points = points
		statics.configFile.setValue("crop_points", points)
	
	def setUseHomography(self, value):
		self.__use_homography = value
		statics.configFile.setValue("use_homography", value)
	
	def warp(self, frame):
		if not self.__use_homography:
			if self.__crop_points:
				p0 = (round(self.__crop_points[0][0]*frame.shape[0]), round(self.__crop_points[0][1]*frame.shape[1]))
				p1 = (round(self.__crop_points[1][0]*frame.shape[0]), round(self.__crop_points[1][1]*frame.shape[1]))
				x  = p0[0]
				xf = p1[0]
				y  = p0[1]
				yf = p1[1]
				return frame[y:yf, x:xf]
			else:
				return frame
		
		homography_matrix = self.getHomography(frame.shape)
		try:
			return cv2.warpPerspective(frame, homography_matrix, (frame.shape[1], frame.shape[0]))
		except:
			return frame
		
	
	def definePoly(self, countor):
		rect = cv2.minAreaRect(countor)
		contourArea = cv2.contourArea(countor)
		rectArea = rect[1][0]*rect[1][1]
		
		return 4 if contourArea/rectArea > 0.75 else 3
	
	def detectarCamisa(self, renderFrame, component_mask):
		# Encontra um contorno para a camisa com base no maior contorno
		mainContours,_ = cv2.findContours(component_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		mainContour = sorted(mainContours, key=cv2.contourArea)[-1]
		
		# Encontra o menor retângulo que se inscreve na camisa
		rectangle = cv2.minAreaRect(mainContour)
		box = cv2.boxPoints(rectangle)
		box = np.int0(box)
		cv2.drawContours(renderFrame, [box], 0, (255,0,0), 2)
		
		# Calcula a posição e ângulo parcial da camisa com base no retângulo
		center = rectangle[0]
		centerMeters = vision.pixel2metric.pixel2meters(center, component_mask.shape)
		angle = rectangle[-1]
		
		return center, centerMeters, angle
		
	def detectarTime(self, img, componentMask, componentTeamMask):
		# Frame a ser renderizado
		renderFrame = img.copy()
		
		# Encontra os contornos internos com área maior que um certo limiar e ordena
		internalContours,_ = cv2.findContours(componentTeamMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_L1)
		internalContours = [countor for countor in internalContours if cv2.contourArea(countor)>10]
		
		countInternalContours = len(internalContours)
			
		# Obtém centro e ângulo da camisa
		center, centerMeters, angle = self.detectarCamisa(renderFrame, componentMask)
		
		# Não é do nosso time
		if countInternalContours == 0:
			return None, centerMeters, angle, renderFrame, center
		
		# Seleciona a forma principal
		mainShape = max(internalContours, key=cv2.contourArea)
		
		# Desenha o contorno no frame
		cv2.drawContours(renderFrame, mainShape, 0, (255,0,0), 1)
		
		# Calcula o centro do contorno principal
		M = cv2.moments(mainShape)
		cX = M["m10"] / M["m00"]
		cY = M["m01"] / M["m00"]
		
		# Calcula o ângulo com base no vetor entre o centro do contorno principal e o centro da camisa
		calculatedAngle = 180.0/np.pi *np.arctan2(-(center[1]-cY), center[0]-cX)
		partialAngles =  -calculatedAngle + self.__angles
		estimatedAngle = partialAngles[np.abs(calculatedAngle - partialAngles).argmin()]
		
		# Define qual o polígono da figura principal
		poligono = self.definePoly(mainShape)
		
		# Computa o identificador com base na forma e no número de contornos internos
		identificador = (0 if poligono == 3 else 2) + countInternalContours -1
		
		# Insere o número do robô no frame
		cv2.putText(renderFrame, str(identificador), (int(center[0])-10, int(center[1])+10), cv2.FONT_HERSHEY_TRIPLEX, 1, (0,0,0))
		
		return identificador, centerMeters, estimatedAngle, renderFrame, center
	
	def segmentarFundo(self, frame):
		img_filtered = cv2.GaussianBlur(frame, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		return cv2.inRange(img_hsv, self.__preto_hsv[0:3], self.__preto_hsv[3:6])
	
	def segmentarTime(self, frame):
		img_filtered = cv2.GaussianBlur(frame, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, self.__preto_hsv[0:3], self.__preto_hsv[3:6])
		return mask & cv2.inRange(img_hsv, self.__time_hsv[0:3], self.__time_hsv[3:6])
	
	def segmentarBola(self, frame):
		img_filtered = cv2.GaussianBlur(frame, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, self.__preto_hsv[0:3], self.__preto_hsv[3:6])
		return mask & cv2.inRange(img_hsv, self.__bola_hsv[0:3], self.__bola_hsv[3:6])
	
	def process(self, frame):
		robosAliadosIdentificados, robosAdversariosIdentificados, bola, processed_image = self.process_frame(frame)
		return robosAliadosIdentificados, robosAdversariosIdentificados, bola, processed_image

	def ui_process(self, frame):
		robosAliadosIdentificados, robosAdversariosIdentificados, bola, processed_image = self.process_frame(frame)
		return processed_image
	
	def converterHSV(self, img):
		img_filtered = cv2.GaussianBlur(img, (5,5), 0)
		return cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
	
	def obterMascaraElementos(self,img):
		return cv2.inRange(img, self.__preto_hsv[0:3], self.__preto_hsv[3:6])
		
	def obterMascaraTime(self, img):
		return cv2.inRange(img, self.__time_hsv[0:3], self.__time_hsv[3:6])
	
	def obterComponentesConectados(self, mask):
		num_components, components = cv2.connectedComponents(mask)
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
		components = cv2.morphologyEx(np.uint8(components), cv2.MORPH_OPEN, kernel)
		kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
		return cv2.dilate(np.uint8(components), kernel, iterations=1)
	
	def process_frame(self, frame):
		# Corta o campo
		img_warpped = self.warp(frame)
		
		# Formata como HSV
		img_hsv = self.converterHSV(img_warpped)
		
		# Segmenta o fundo
		mask = self.obterMascaraElementos(img_hsv)
		
		# Segmenta o time
		teamMask = self.obterMascaraTime(img_hsv)
		
		# Encontra componentes conectados e aplica operações de abertura e dilatação
		components = self.obterComponentesConectados(mask)
		
		# Frame zerado a ser renderizado
		processed_image = np.zeros(img_warpped.shape, np.uint8)
		
		# Desenha lado do campo
		height, width, _ = processed_image.shape
		self.draw_left_rectangle(processed_image, (0,255,0) if world.fieldSide == field.LEFT else (0,0,255))
		self.draw_right_rectangle(processed_image, (0,255,0) if world.fieldSide == field.RIGHT else (0,0,255))
		cv2.line(processed_image, (int(width/2),0), (int(width/2),height), (100,100,100), 1)
		
		# Listas com aliados e inimigos
		robosAliados = [(i,(0,0),0,False,(0,0)) for i in range(5)]
		robosAdversariosIdentificados = []
		
		# Itera por cada elemento conectado
		for label in np.unique(components)[1:]:
			# Máscara do elemento
			componentMask = np.uint8(np.where(components == label, 255, 0))
			
			# Imagem com apenas o elemento
			comp = cv2.bitwise_and(img_warpped, img_warpped, mask=componentMask)
			
			# Máscara dos componentes internos do elemento
			componentTeamMask = componentMask & teamMask
			
			# Tenta detectar o elemento como sendo do nosso time ou adversário
			identificador, centro, angulo, component_image, centroPixels = self.detectarTime(comp, componentMask, componentTeamMask)
			
			# Adiciona às listas de aliados ou inimigos
			if identificador is not None and identificador < 5:
				robosAliados[identificador] = (identificador, centro, angulo, True, centroPixels)
			else:
				robosAdversariosIdentificados.append((centro, angulo))
				
			if component_image is not None:
				processed_image = cv2.add(processed_image, component_image)
				
		# Segmenta a bola
		bolaMask = mask & cv2.inRange(img_hsv, self.__bola_hsv[0:3], self.__bola_hsv[3:6])
		bolaContours,_ = cv2.findContours(bolaMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		bolaContours = [countor for countor in bolaContours if cv2.contourArea(countor)>10]
		
		if len(bolaContours) != 0:
			bolaContour = max(bolaContours, key=cv2.contourArea)
			((x,y), radius) = cv2.minEnclosingCircle(bolaContour)
			cv2.circle(processed_image, (int(x),int(y)), int(radius), (0,255,0), 1)
			
			bola = (vision.pixel2metric.pixel2meters((x,y), bolaMask.shape), radius)
		else: bola = None
		
		self.atualizarRobos(robosAliados, robosAdversariosIdentificados, bola)

		return robosAliados, robosAdversariosIdentificados, bola, processed_image
	
	def draw_left_rectangle(self, image, color, thickness=10):
		height, width, _ = image.shape
		cv2.line(image, (int(width/2)-thickness,0), (0,0), color, thickness)
		cv2.line(image, (0,0), (0, height), color, thickness)
		cv2.line(image, (int(width/2)-thickness,height), (0, height), color, thickness)
		
	def draw_right_rectangle(self, image, color, thickness=10):
		height, width, _ = image.shape
		cv2.line(image, (int(width/2)+thickness,0), (width,0), color, thickness)
		cv2.line(image, (width,0), (width, height), color, thickness)
		cv2.line(image, (width, height), (int(width/2)+thickness, height), color, thickness)
