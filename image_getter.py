"""
Modul se sastoji od funkcionalnosti potrebnih za dohvat slike sa
rostopic-a te obradu slike koja ukljucuje pronalazenje markera,
trazenja koordinata i sl.
"""

import roslib
import rospy
import numpy
import cv2
import cv2.cv as cv
import cv_bridge as cvb
from linear import Circle, Triangle, Rectangle, Line
from sensor_msgs.msg import Image


NODE_NAME = "camera"					# ime node-a
SUB_NAME = "/ardrone/front/image_raw"	# ime subscribera
CAMERA_SIZE = (320, 240)

_nodeExists = False			# govori jel node vec kreiran

# ovdje su uz glavne razine boje izmjerene eksperimentalno.
# ovo je podlozno promjenama u ovisnosti o osvjetljenju i okolini
RED_BGR = (60, 60, 220)
GREEN_BGR = (50, 140, 60)
BLUE_BGR = (150, 100, 30)
ORANGE_BGR = (20, 100, 210)
YELLOW_BGR = (80, 180, 180)
PURPLE_BGR = (0, 0, 0)	## NIJE DEFINIRANO
TRESHOLD = 50		# prag za odredjivanje boje

# HSV boje, treba provjerit ih tu i tamo
YELL_MIN = numpy.array([15, 115, 110], numpy.uint8)		# veliki ntervali!!
YELL_MAX = numpy.array([30, 200, 210], numpy.uint8)
GREEN_MIN = numpy.array([40, 130, 90], numpy.uint8)
GREEN_MAX = numpy.array([80, 230, 180], numpy.uint8)
RED1_MIN = numpy.array([0, 150, 140], numpy.uint8)		# za crvenu treba obe
RED1_MAX = numpy.array([1, 250, 250], numpy.uint8)		# vrijednosti poslat
RED2_MIN = numpy.array([174, 150, 140], numpy.uint8)
RED2_MAX = numpy.array([180, 250, 250], numpy.uint8)
BLUE_MIN = numpy.array([97, 160, 70], numpy.uint8)
BLUE_MAX = numpy.array([107, 250, 150], numpy.uint8)
ORANGE_MIN = numpy.array([1, 160, 130], numpy.uint8)
ORANGE_MAX = numpy.array([14, 240, 240], numpy.uint8)
PURPLE_MIN = numpy.array([0, 60, 80], numpy.uint8)
PURPLE_MAX = numpy.array([20, 120, 135], numpy.uint8)

YELLOW = (YELL_MIN, YELL_MAX)
GREEN = (GREEN_MIN, GREEN_MAX)
BLUE = (BLUE_MIN, BLUE_MAX)
RED1 = (RED1_MIN, RED1_MAX)
RED2 = (RED2_MIN, RED2_MAX)
ORANGE = (ORANGE_MIN, ORANGE_MAX)
PURPLE = (PURPLE_MIN, PURPLE_MAX)


def init_node(nodeName = NODE_NAME):
	"""Inicijalizira node."""
	
	global _nodeExists
	if not _nodeExists:		# napravi node ako ne postoji vec
		rospy.init_node(nodeName)
		_nodeExists = True


def newListener(image_callback, subName = SUB_NAME):
	"""
	Listener za dohvat slike sa kamere na letjelici.
	nodeName - ime noda
	subName - ime za subscriber
	image_callback - funkcija koja se zove za izvrsavanje
	"""
	
	global _nodeExists
	if not _nodeExists:
		raise StandardError("Node nije stvoren")
	
	rospy.Subscriber(subName, Image, image_callback)


def rospySpin():
	"""
	Pozove rospy.spin(), koristi se kako bi se moglo vise
	subscribera napravit.
	"""
	
	rospy.spin()
		
			
class RospyImage:
	"""Razred za sliku u rospy formatu."""

	FORMAT = "bgr8"		# format slike za pretvaranje preko bridge-a

	def __init__(self, imageMsg):
		"""Prima image iz ros formata."""
		
		self._rospyImage = imageMsg
		self._numpyArray = self._convertToNumpy(imageMsg)
		
		
	def _convertToNumpy(self, imageMsg):
		"""
		Konvertira sliku iz formata koji dobiva iz rospy-a u numpy array,
		format potreban za opencv.
		"""
	
		bridge = cvb.CvBridge()
		try:
			cv_image = bridge.imgmsg_to_cv(imageMsg, self.FORMAT)
		except cvb.CvBridgeError, e:
			print e
		
		img = numpy.asarray(cv_image)
		return img
		
	
	def getNumpyArray(self):
		"""Getter za sliku u opencv formatu (numpy array)."""
		
		return self._numpyArray
		
		
	def createNumpyImage(self):
		"""Getter za sliku kao NumpyImage."""
		
		return NumpyImage(self._numpyArray)
		
	
class NumpyImage:
	"""Osnovna obrada slike."""

	BLUR = 5		# konstanta za zamagljenje slike
	DP = 1			# konstanta za trazenje krugova (ne znam tocno cemu sluzi)
	MINDIST = 50		# najmanja udaljenost izmedju 2 kruga na slici
	EDGEDET = 50		# konstanta za rubove (ne znam tocno cemu sluzi)
	CENTERDET = 30		# isto neka konstanta za krugove
	MINRAD = 0 			# najmanji moguci radius kruga
	MAXRAD = 60			# najveci moguci radius kruga
	
	POLYGON_TH_L = 50		# donji treshold za canny algoritam
	POLYGON_TH_H = 200		# gornji treshold za canny algoritam
	EPSILON_CONST = 0.04	# za trazenje n-terokuta (veci broj vise n-terokuta)

	LINE_TH_L = 50			# donji treshold za canny za linije
	LINE_TH_H = 200			# gornji treshold za canny za linije
	RHO = 1
	THETA = cv.CV_PI / 180
	HOUGH_THRESHOLD = 50
	MIN_LINE_LENGTH = 50
	MAX_LINE_GAP = 20


	def __init__(self, numpyArray):
		"""Prima numpy array."""
	
		self._image = numpyArray
		(self._width, self._height) = cv.GetSize(cv.fromarray(self._image))
		self.XC = int(self._width / 2)		# srediste x osi
		self.YC = int(self._height / 2)		# srediste y osi

		self._hsv = cv2.cvtColor(self._image, cv2.COLOR_BGR2HSV) 
		
		
	def getDimensions(self):
		"""Vraca dimenzije slike (sirina, visina)."""
		
		return (self._width, self._height)
		
		
	def getNumpyArray(self):
		"""Vraca numpy array."""
		
		return self._image


	def getHSV(self):
		"""Vrati HSV sliku."""

		return self._hsv
		
	
	def getPixelAtBGR(self, x, y):
		"""Vraca vrijednost boje na pixelu (x, y) u obliku (b, g, r)."""
		
		bgr = self._image[y][x]
		b, g, r = bgr[0], bgr[1], bgr[2]
		
		return b, g, r


	def getPixelAt(self, x, y):
		"""Dohvati pixel u HSV formatu."""

		return self._hsv[y][x]

		
	def findCircles(self, centerdet = CENTERDET):
		"""
		Pronalazi krugove na slici.
		Vraca listu koordinata sredista i radiusa.
		Ako ne pronadje niti jedan krug, vraca None.
		"""
		
		imgTemp = cv2.medianBlur(self._image, self.BLUR)
		imgTemp = self._convertToGray()
		circles = cv2.HoughCircles(imgTemp,
			cv.CV_HOUGH_GRADIENT,
			self.DP,
			self.MINDIST,
			param1 = self.EDGEDET,
			param2 = centerdet,
			minRadius = self.MINRAD,
			maxRadius = self.MAXRAD)
		
		if circles == None:
			return None
		
		circles = numpy.uint16(numpy.around(circles))
		linearCircles = self._convertCircles(circles)
		return linearCircles
		
	
	def _convertToGray(self):
		"""Pretvori sliku u sivu i vrati novu instancu numpy array-a."""
		
		return cv2.cvtColor(self._image, cv2.COLOR_BGR2GRAY) 
		
		
	def _convertCircles(self, circles):
		"""
		Pretvori krugove iz tipa koji nadje opencv u
		objekte razreda Circle u modulu linear.
		"""
		
		linearCircles = []
		for c in circles[0,:]:
			x, y = self._translateCoordinates(c[0], c[1])
			linearCircles += [Circle(c[2], (x, y))]

		return linearCircles


	def markCircles(self, circles):
		"""Oznaci sve krugove na slici. I vrati numpy array."""
		
		if circles == None:
			return self._image
			
		for c in circles:
			x, y = self._reverseCoordinates(c.x, c.y)
			if x >= self._width or y >= self._height or x < 0 or y < 0:
				continue
			# vanjski krug
			cv2.circle(self._image, (x, y), c.radius, GREEN_BGR, 2)
			# sredina kruga
			cv2.circle(self._image, (x, y), 2, RED_BGR, 3)
		return self._image
		
		
	def filterColoredCircles(self, circles, color):
		"""Filtrira listu krugova po boji. Vraca None ako nema te boje"""
		
		if circles == None:
			return None
		
		filtered = []
		for c in circles:
			x, y = self._reverseCoordinates(c.x, c.y)
			pixelColor = self.getPixelAt(x, y)
			if NumpyImage._isRightColor(pixelColor, color):
				filtered += [c]

		return filtered
		
		
	@staticmethod
	def _isRightColorBGR(pixelColor, color):
		"""
		Usporedjuje boju pixela, s zadanom bojom.
		pixelColor - boja koja se nalazi na nekom pixelu
		color - zadana boja za usporedbu
		"""

		b, g, r = pixelColor
		cb, cg, cr = color
		return b >= cb - TRESHOLD and b < cb + TRESHOLD		\
			and g >= cg - TRESHOLD and g < cg + TRESHOLD		\
			and r >= cr - TRESHOLD and r < cr + TRESHOLD
		

	@staticmethod
	def _isRightColor(pixelColor, hsvRange):
		"""Provjeri boju koja je u HSV formatu."""

		c1, c2, c3 = pixelColor
		hl1, hl2, hl3 = hsvRange[0]
		hh1, hh2, hh3 = hsvRange[1]
		return c1 >= hl1 and c1 < hh1		\
			and c2 >= hl2 and c2 < hh2		\
			and c3 >= hl3 and c3 < hh3

			
	def _translateCoordinates(self, x, y):
		"""
		Prima apsolutne koordinate i translatira ih u relativne.
		Apsolutne koordinate imaju (0,0) u gornjem lijevom uglu
		i (width - 1, height - 1) u donjem desnom. Relativne se
		racunaju u odnosu na centar slike.
		"""
		
		return x - self.XC, self.YC - y

		
	def _reverseCoordinates(self, x, y):
		"""Vraca koordinate u one potrebne za crtanje na slici."""
	
		return x + self.XC, self.YC - y
		
		
	def findPolygon(self, vertices=3, epsilon_const=EPSILON_CONST):
		"""Trazi n-terokut sa zadanim brojem vrhova."""
		
		polygons = []
		
		for gray in cv2.split(self._image):
			# pronadje linije
			canny = cv2.Canny(gray, self.POLYGON_TH_L, self.POLYGON_TH_H)
			# konture pronadje
			contours, _ = cv2.findContours(canny,
				cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
				
			for cnt in contours:
				epsilon = epsilon_const * cv2.arcLength(cnt, True)
				approx = cv2.approxPolyDP(cnt, epsilon, True)
				
				if len(approx) == vertices:
					translated = self._translateAll(approx)
					
					if vertices == 3:		# trokut
						poly = Triangle(cnt, translated)	
					elif vertices == 4:		# pravokutnik
						poly = Rectangle(cnt, translated)
					else:
						raise NotImplementedError("Nije implementirano")
						
					polygons += [poly]

		return polygons
		
		
	def _translateAll(self, coordinates):
		"""Translatira listu koordinata. Koristi se za trokut."""
	
		if coordinates == None:
			return []
			
		translated = []
		for c in coordinates:
			translated += [self._translateCoordinates(c[0,0], c[0,1])]
			
		return translated
		
	
	def markPolygons(self, polygons):
		"""Oznaci vise n-terokuta na slici."""
		
		if polygons == None:
			return self._image
		
		for polygon in polygons:
			self._markSinglePolygon(polygon)
		return self._image
	
		
	def _markSinglePolygon(self, polygon):
		"""Oznaci n-terokut na slici."""
				
		# oznaci stranice
		contours = polygon.getContours()
		for cnt in contours:
			cv2.drawContours(self._image, [cnt], 0, GREEN_BGR, 2)
		
		# oznaci vrhove
		vertices = polygon.getVertices()
		for vertex in vertices:
			x, y = self._reverseCoordinates(vertex[0], vertex[1])
			cv2.circle(self._image, (x, y), 5, BLUE_BGR, -1)


	def filterColoredPolygons(self, polygons, color):
		"""Filtrira n-terokute odredjene boje."""
		
		if polygons == None:
			return None
			
		filtered = []
		for p in polygons:
			xc, yc = p.findCenter()
			x, y = self._reverseCoordinates(xc, yc)
			pixelColor = self.getPixelAt(x, y)
			if NumpyImage._isRightColor(pixelColor, color):
				filtered += [p]
				
		return filtered


	def findLines(self):
		"""Pronadje sve linije na slici."""

		gray = self._convertToGray()
		canny = cv2.Canny(gray, self.LINE_TH_L, self.LINE_TH_H)
		lines = cv2.HoughLinesP(canny,
			rho = self.RHO,
			theta = self.THETA,
			threshold = self.HOUGH_THRESHOLD,
			minLineLength = self.MIN_LINE_LENGTH,
			maxLineGap = self.MAX_LINE_GAP)

		if lines is None:
			return None
		f = self._translateCoordinates
		return map(lambda l: Line(f(l[0], l[1]), f(l[2], l[3])), lines[0])


	def markLines(self, lines):
		"""Oznaci linije na slici."""

		if lines is None:
			return self._image

		for l in lines:
			a = self._reverseCoordinates(l.a[0], l.a[1])
			b = self._reverseCoordinates(l.b[0], l.b[1])
			cv2.line(self._image, a, b, RED_BGR, 3)
		return self._image
			

	#deprecated
	def findColorPolygons(self, hsvMin, hsvMax, vertices=3):
		"""Ovo je bila proba sa HSV-om."""

		frameThreshold = cv2.inRange(self._hsv, hsvMin, hsvMax)
		# pronadje linije
		canny = cv2.Canny(frameThreshold, self.POLYGON_TH_L, self.POLYGON_TH_H)
		# konture pronadje
		contours, _ = cv2.findContours(canny,
			cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		polygons = []
			
		for cnt in contours:
			epsilon = self.EPSILON_CONST * cv2.arcLength(cnt, True)
			approx = cv2.approxPolyDP(cnt, epsilon, True)
			
			if len(approx) == vertices:
				translated = self._translateAll(approx)
				
				if vertices == 3:		# trokut
					poly = Triangle(cnt, translated)	
				elif vertices == 4:		# pravokutnik
					poly = Rectangle(cnt, translated)
				else:
					raise NotImplementedError("Nije implementirano")
					
				polygons += [poly]

		return polygons