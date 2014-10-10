import cv2
import numpy as np
import image_getter as ig


WINDOW_NAME_REGULAR = "stream"
WINDOW_NAME_CIRCLES = "circles"
WINDOW_NAME_TRIANGLES = "triangles"
WINDOW_NAME_RECTANGLES = "rectangles"
WINDOW_NAME_LINES = "lines"


def colorCallback(image):
	"""Ispise hsv od centralnog pixela."""
	
	rosImg = ig.RospyImage(image)
	npimg = rosImg.createNumpyImage()
	width, height = ig.CAMERA_SIZE
	#height /= 3
	width += 250
	print npimg.getPixelAt(width / 2, height / 2)
	cv2.circle(npimg.getNumpyArray(), (width/2, height/2), 2, (0,0,0), 2)
	cv2.imshow("boja", npimg.getNumpyArray())
	cv2.waitKey(1)


def streamCallback(image):
	"""
	Funkcija se koristi kao callback za subscriber-a na
	/ardrone/front/image_raw.
	Prikazuje stream u prozoru.
	"""
	
	rosImg = ig.RospyImage(image)
	npImg = rosImg.createNumpyImage()
	cv2.imshow(WINDOW_NAME_REGULAR, npImg.getNumpyArray())
	cv2.waitKey(1)
	
	
def circlesCallback(image):
	"""
	Funkcija se koristi kao callback za subscriber-a na
	/ardrone/front/image_raw.
	Prikazuje stream u prozoru i oznacava krugove.
	"""
	
	rosImg = ig.RospyImage(image)
	npimg = rosImg.createNumpyImage()
	circles = npimg.findCircles()
	cimg = npimg.markCircles(circles)
	cv2.imshow(WINDOW_NAME_CIRCLES, cimg)
	cv2.waitKey(1)
	
	
def circlesCoordinatesCallback(image):
	"""Pronalazi krugove i vraca koordinate njihovog sredista."""
	
	rosImg = ig.RospyImage(image)
	npImg = rosImg.createNumpyImage()
	circles = npImg.findCircles()
	print circles
	

def colorCirclesCallback(image):
	"""
	Pronadje i oznaci crveni krug na streamu.
	Ispisuje koordinate sredista oznacenog kruga.
	Napomena: pronalazi samo jedan krug.
	"""
	
	rosImg = ig.RospyImage(image)
	npimg = rosImg.createNumpyImage()
	circles = npimg.findCircles()
	circles = npimg.filterColoredCircles(circles, ig.RED)
	npimg.markCircles(circles)
	cv2.imshow(WINDOW_NAME_CIRCLES, npimg.getNumpyArray())
	cv2.waitKey(1)
				
			
def triangleCallback(image):
	"""Pronalazi i oznacuje trokute na slici."""
	
	rosImg = ig.RospyImage(image)
	npimg = rosImg.createNumpyImage()
	triangles = npimg.findPolygon(3)
	img = npimg.markPolygons(triangles)
	cv2.imshow(WINDOW_NAME_TRIANGLES, img)
	cv2.waitKey(1)
	
	
def colorTriangleCallback(image):
	"""Pronalazi trokute odredjene boje."""
	
	rosImg = ig.RospyImage(image)
	npimg = rosImg.createNumpyImage()
	triangles = npimg.findPolygon(3)
	coloredTriangles = npimg.filterColoredPolygons(triangles, ig.RED)	
	
	if coloredTriangles != None and len(coloredTriangles) > 1:
		triangle = coloredTriangles[0]
	else:
		cv2.imshow(WINDOW_NAME_TRIANGLES, npimg.getNumpyArray())
		cv2.waitKey(1)
		return
	
	npimg.markPolygons([triangle])
	center = triangle.findCenter()	
	direction = triangle.findDirection()
	
	print center, direction
	
	cv2.imshow(WINDOW_NAME_TRIANGLES, npimg.getNumpyArray())
	cv2.waitKey(1)	
		

def rectangleCallback(image):
	"""Pronalazi cetverokute na slici."""
	
	npimg = ig.RospyImage(image).createNumpyImage()
	rectangles = npimg.findPolygon(4)
	npimg.markPolygons(rectangles)
	cv2.imshow(WINDOW_NAME_RECTANGLES, npimg.getNumpyArray())
	cv2.waitKey(1)
	
	
def colorRectangleCallback(image):
	"""Pronalazi pravokutnik odredjene boje."""
	
	npimg = ig.RospyImage(image).createNumpyImage()
	rectangles = npimg.findPolygon(4)
	colored = npimg.filterColoredPolygons(rectangles, ig.YELLOW)
	
	if colored != None and len(colored) > 1:
		rect = colored[0]
	else:
		cv2.imshow(WINDOW_NAME_RECTANGLES, npimg.getNumpyArray())
		cv2.waitKey(1)
		return;
		
	npimg.markPolygons([rect])
	center = rect.findCenter()
	print center
	
	cv2.imshow(WINDOW_NAME_RECTANGLES, npimg.getNumpyArray())
	cv2.waitKey(1)
		

def linesStreamCallback(image):
	"""Pronadje sve linije na slici i oznaci ih."""

	npimg = ig.RospyImage(image).createNumpyImage()
	lines = npimg.findLines()
	npimg.markLines(lines)
	cv2.imshow(WINDOW_NAME_LINES, npimg.getNumpyArray())
	cv2.waitKey(1)


def linesCallback(image):
	"""Pronadje sve linije na slici. Ispise neke podatke vezane za njih."""

	npimg = ig.RospyImage(image).createNumpyImage()
	lines = npimg.findLines()
	vectors = None
	if lines is not None:
		vectors = map(lambda l: l.vector().normalize(), lines)
	print vectors


def refLineCallback(image):
	"""Pronadje liniju (zid labirinta) po kojoj usmjerava letjelicu."""

	npimg = ig.RospyImage(image).createNumpyImage()
	lines = npimg.findLines()
	if lines is not None:
		maxLen = max(map(lambda l: l.length(), lines))
		line = filter(lambda l: abs(maxLen - l.length()) < 0.001, lines)
		npimg.markLines(line)
	cv2.imshow(WINDOW_NAME_LINES, npimg.getNumpyArray())
	cv2.waitKey(1)


def yellowHSV(image):
	"""Sluzilo za testiranje HSV boja."""

	npimg = ig.RospyImage(image).createNumpyImage()
	rectangles = npimg.findColorPolygons(ig.YELL_MIN, ig.YELL_MAX, 4)
	if rectangles is not None and len(rectangles) > 0:
		npimg.markPolygons([rectangles[0]])
	cv2.imshow(WINDOW_NAME_RECTANGLES, npimg.getNumpyArray())
	cv2.waitKey(1)

	
def destroyAllWindows():
	"""Zatvori sve prozore vezane za OpenCV."""
	
	cv2.destroyAllWindows()
