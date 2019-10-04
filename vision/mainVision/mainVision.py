import cv2
import statics.configFile
import numpy as np
import vision.pixel2metric
import vision.mainVision.frameRenderer
import vision.vision
import gui.mainWindow
from statics.static_classes import world
from statics import field
import math

class Robo():
	def __init__(self, identificador):
		self.identificador = identificador
		self.centro = (-1,-1)
		self.centroPixels = (-1,-1)
		self.angulo = -1
		self.estado = "Não-Identificado"
		self.ui = None

class MainVision(vision.vision.Vision):
	def __init__(self, parent):
		self.__posicaoIdentificador = [(None,None) for i in range(world.number_of_robots)]
		self.__n_robos = 2*world.number_of_robots
		self.__angles = np.array([0, 90, 180, -90, -180])
		self.__homography = None
		self.__default_preto_hsv = [0,94,163,360,360,360]
		self.__default_time_hsv = [13,0,0,32,360,360]
		self.__default_bola_hsv = [0, 117, 0, 98, 360, 360]
		self.__current_frame_shape = None
		self.__stability_use_current = False
		super().__init__(parent)

	def config_init(self):
		self.__preto_hsv = np.array(statics.configFile.getValue("preto_hsv_interval", self.__default_preto_hsv))
		self.__time_hsv = np.array(statics.configFile.getValue("time_hsv_interval", self.__default_time_hsv))
		self.__bola_hsv = np.array(statics.configFile.getValue("bola_hsv_interval", self.__default_bola_hsv))
		self.__homography = np.array(statics.configFile.getValue("homography_matrix"), None)
		self.__use_homography = statics.configFile.getValue("use_homography", True)
		self.__crop_points = statics.configFile.getValue("crop_points")
		self.__homography_points = statics.configFile.getValue("homography_points")
		self.__cont_rect_area_ratio = statics.configFile.getValue("cont_rect_area_ratio", 0.75)
		self.__min_internal_area_contour = statics.configFile.getValue("min_internal_area_contour", 10)
		self.__stability_param = statics.configFile.getValue("stability_param", 0.99)


	def ui_init(self):
		self.frameRenderers = {
			"cortarCampo": vision.mainVision.frameRenderer.cortarCampo(self),
			"segmentarFundo": vision.mainVision.frameRenderer.segmentarPreto(self),
			"segmentarTime": vision.mainVision.frameRenderer.segmentarTime(self),
			"segmentarBola": vision.mainVision.frameRenderer.segmentarBola(self),
			"parametrosVisao": vision.mainVision.frameRenderer.parametrosVisao(self),
			"identificarRobos": vision.mainVision.frameRenderer.identificarRobos(self),
		}
		#self.selectedFrameRenderer = self.frameRenderers[0]
		#gui.mainWindow.MainWindow().set_frame_renderer(0)
	
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
	
	@property
	def areaRatio(self):
		return self.__cont_rect_area_ratio
		
	@property
	def minInternalAreaContour(self):
		return self.__min_internal_area_contour
		
	@property
	def stabilityParam(self):
		return self.__stability_param
	
	def usarIdentificadorAtual(self):
		self.__stability_use_current = True
	
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
	
	def atualizarAreaRatio(self, value):
		self.__cont_rect_area_ratio = value
		statics.configFile.setValue("cont_rect_area_ratio", value)
	
	def atualizarMinInternalArea(self, value):
		self.__min_internal_area_contour = value
		statics.configFile.setValue("min_internal_area_contour", value)
	
	def atualizarParametroEstabilidade(self, value):
		self.__stability_param = value
		statics.configFile.setValue("stability_param", value)
	
	def getHomography(self, shape):
		if self.__current_frame_shape != shape:
			if self.__homography_points is None:
				return None
			self.updateHomography(self.__homography_points, shape)
		return self.__homography
	
	def updateHomography(self, points, shape):
		height, width, _ = shape
		self.__current_frame_shape = shape
		self.__homography_points = points
		key_points = np.array(points) * np.array([height, width])
		frame_points = np.array(sorted([[0,0],[0, height],[width,0],[width,height]], key=sum))
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
	
	def obterIdentificador(self, center, candidate):
		if self.__stability_use_current is True:
			self.__posicaoIdentificador = [(None,None) for i in range(world.number_of_robots)]
			self.__stability_use_current = False
		
		minDistance = math.inf
		nearestIdx = None
		for idx,(pos,id) in enumerate(self.__posicaoIdentificador):
			if pos is None:
				nearestIdx = idx
				break
			
			distance = abs(pos[0]-center[0])+abs(pos[1]-center[1])
			if distance < minDistance:
				minDistance = distance
				nearestIdx = idx
		
		if nearestIdx is None:
			return candidate
		
		oldMean = self.__posicaoIdentificador[nearestIdx][1]

		if oldMean is None:
			newMean = candidate
		else:
			newMean = self.__stability_param*oldMean+(1-self.__stability_param)*candidate
		
		self.__posicaoIdentificador[nearestIdx] = (center,newMean)
		
		return round(newMean)
		
	
	def definePoly(self, countor):
		rect = cv2.minAreaRect(countor)
		contourArea = cv2.contourArea(countor)
		rectArea = rect[1][0]*rect[1][1]
		
		return 4 if contourArea/rectArea > self.__cont_rect_area_ratio else 3
	
	def detectarCamisa(self, renderFrame, component_mask):
		# Encontra um contorno para a camisa com base no maior contorno
		mainContours,_ = cv2.findContours(component_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		mainContour = sorted(mainContours, key=cv2.contourArea)[-1]
		
		# Encontra o menor retângulo que se inscreve no contorno
		rectangle = cv2.minAreaRect(mainContour)
		
		# Calcula a posição e ângulo parcial da camisa com base no retângulo
		center = rectangle[0]
		centerMeters = vision.pixel2metric.pixel2meters(center, component_mask.shape)
		angle = rectangle[-1]
		
		return center, centerMeters, angle, mainContour
		
	def detectarTime(self, renderFrame, componentTeamMask, center, rectangleAngle):
		
		# Encontra os contornos internos com área maior que um certo limiar e ordena
		internalContours,_ = cv2.findContours(componentTeamMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_L1)
		internalContours = [countor for countor in internalContours if cv2.contourArea(countor)>=self.__min_internal_area_contour]
		
		countInternalContours = len(internalContours)
		
		# Não é do nosso time
		if countInternalContours == 0:
			return None
		
		# Seleciona a forma principal
		mainShape = max(internalContours, key=cv2.contourArea)
		
		# Desenha o contorno no frame
		cv2.drawContours(renderFrame, internalContours, -1, (255,255,255), 1)
		
		# Calcula o centro do contorno principal
		M = cv2.moments(mainShape)
		cX = M["m10"] / M["m00"]
		cY = M["m01"] / M["m00"]
		
		# Calcula o ângulo com base no vetor entre o centro do contorno principal e o centro da camisa
		calculatedAngle = 180.0/np.pi *np.arctan2(-(center[1]-cY), center[0]-cX)
		partialAngles =  -rectangleAngle + self.__angles
		estimatedAngle = partialAngles[np.abs(calculatedAngle - partialAngles).argmin()]
		
		# Define qual o polígono da figura principal
		poligono = self.definePoly(mainShape)
		
		# Computa o identificador com base na forma e no número de contornos internos
		candidato = (0 if poligono == 3 else 2) + countInternalContours -1
		if candidato >= self.__n_robos: return None
		
		identificador = self.obterIdentificador(center, candidato)
		
		return identificador, estimatedAngle
	
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
		robos, bola, processed_image = self.process_frame(frame)
		return robos[0:self.__n_robos], robos[self.__n_robos:2*self.__n_robos], bola, processed_image

	def ui_process(self, frame):
		robos, bola, processed_image = self.process_frame(frame)
		return robos[0:self.__n_robos], robos[self.__n_robos:2*self.__n_robos], bola, processed_image
	
	def converterHSV(self, img):
		img_filtered = cv2.GaussianBlur(img, (5,5), 0)
		return cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
	
	def obterMascaraElementos(self,img):
		return cv2.inRange(img, self.__preto_hsv[0:3], self.__preto_hsv[3:6])
		
	def obterMascaraTime(self, img):
		return cv2.inRange(img, self.__time_hsv[0:3], self.__time_hsv[3:6])
		
	def obterMascaraBola(self, img):
		return cv2.inRange(img, self.__bola_hsv[0:3], self.__bola_hsv[3:6])
	
	def obterComponentesConectados(self, mask):
		num_components, components = cv2.connectedComponents(mask)
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
		components = cv2.morphologyEx(np.uint8(components), cv2.MORPH_OPEN, kernel)
		kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
		dilated = cv2.erode(np.uint8(components), kernel, iterations=1)
		return [np.uint8(np.where(dilated == label, 255, 0)) for label in np.unique(dilated)[1:]]
	
	def identificarBola(self, frameToDraw, mask):
		bolaContours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		bolaContours = [countor for countor in bolaContours if cv2.contourArea(countor) >= self.__min_internal_area_contour]

		if len(bolaContours) != 0:
			bolaContour = max(bolaContours, key=cv2.contourArea)
			((x,y), radius) = cv2.minEnclosingCircle(bolaContour)
			cv2.circle(frameToDraw, (int(x),int(y)), int(radius), (255,0,0), 2)
			
			return (vision.pixel2metric.pixel2meters((x,y), mask.shape), radius)
		
		else: return None
	
	def process_frame(self, frame):
		if self.__n_robos != world.number_of_robots:
			self.__posicaoIdentificador = [(None,None) for i in range(world.number_of_robots)]
			self.__n_robos = world.number_of_robots

		# Corta o campo
		img_warpped = self.warp(frame)
		
		# Formata como HSV
		img_hsv = self.converterHSV(img_warpped)
		
		# Segmenta o fundo
		mask = self.obterMascaraElementos(img_hsv)
		
		# Segmenta o time
		teamMask = self.obterMascaraTime(img_hsv)
		
		# Segmenta a bola
		bolaMask = self.obterMascaraBola(img_hsv)
		
		# Encontra componentes conectados e aplica operações de abertura e dilatação
		components = self.obterComponentesConectados(mask)
		
		# Frame zerado a ser renderizado
		processed_image = np.zeros(img_warpped.shape, np.uint8)
		for c in components:
			processed_image = cv2.add(processed_image, cv2.bitwise_and(img_warpped, img_warpped, mask=c))
		
		# Desenha lado do campo
		self.draw_left_rectangle(processed_image, (0,255,0) if world.fieldSide == field.LEFT else (0,0,255))
		self.draw_right_rectangle(processed_image, (0,255,0) if world.fieldSide == field.RIGHT else (0,0,255))
		self.draw_middle_line(processed_image)
		
		# Listas com aliados e inimigos e bola
		robos = [(i,(0,0),0,False,(0,0)) for i in range(2*self.__n_robos)]
		bola = None
		advId = self.__n_robos
		
		# Itera por cada elemento conectado
		for componentMask in components:
			# Máscara com a bola
			componentBolaMask = componentMask & bolaMask
			
			b,_ = cv2.findContours(componentMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			cv2.drawContours(processed_image, b, -1, (255,255,255), 1)
			
			# Tenta identificar uma bola
			bola_ = self.identificarBola(processed_image, componentBolaMask)
			if bola_ is not None:
				bola = bola_
				continue
				
			# Obtém dados do componente como uma camisa
			centro, centerMeters, angulo, camisaContour = self.detectarCamisa(processed_image, componentMask)
			
			# Máscara dos componentes internos do elemento
			componentTeamMask = componentMask & teamMask
			
			# Tenta identificar um aliado
			aliado = self.detectarTime(processed_image, componentTeamMask, centro, angulo)
			if aliado is not None:
				robos[aliado[0]] = (aliado[0], centerMeters, aliado[1], True, centro)
				cv2.putText(processed_image, str(aliado[0]), (int(centro[0])-10, int(centro[1])+10), cv2.FONT_HERSHEY_TRIPLEX, 1, (0,0,0))
				self.draw_contour_rectangle(processed_image, camisaContour, (0,255,0))
				continue
			
			# Adiciona camisa como adversário
			if advId < 2*self.__n_robos:
				robos[advId] = (advId, centerMeters, angulo, True, centro)
				cv2.putText(processed_image, str(advId), (int(centro[0])-10, int(centro[1])+10), cv2.FONT_HERSHEY_TRIPLEX, 1, (0,0,0))
				self.draw_contour_rectangle(processed_image, camisaContour, (0,0,255))
				advId = advId + 1
			
		return robos, bola, processed_image
	
	def draw_left_rectangle(self, image, color, thickness=5):
		height, width, _ = image.shape
		cv2.line(image, (int(width/2)-thickness,0), (0,0), color, thickness)
		cv2.line(image, (0,0), (0, height), color, thickness)
		cv2.line(image, (int(width/2)-thickness,height), (0, height), color, thickness)
		
	def draw_right_rectangle(self, image, color, thickness=5):
		height, width, _ = image.shape
		cv2.line(image, (int(width/2)+thickness,0), (width,0), color, thickness)
		cv2.line(image, (width,0), (width, height), color, thickness)
		cv2.line(image, (width, height), (int(width/2)+thickness, height), color, thickness)
	
	def draw_middle_line(self, image):
		height, width, _ = image.shape
		cv2.line(image, (int(width/2),0), (int(width/2),height), (100,100,100), 1)
	
	def draw_contour_rectangle(self, image, contour, color=(255,0,0)):
		# Encontra o menor retângulo que se inscreve no contorno
		rectangle = cv2.minAreaRect(contour)
		box = cv2.boxPoints(rectangle)
		box = np.int0(box)
		cv2.drawContours(image, [box], 0, color, 2)
