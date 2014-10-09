"""
Ovo je modul koji sluzi za pronalazenje puta kroz labirint koristenjem
AStar (A*) algoritma. 
"""

import AStar
import cv2
import cv2.cv as cv
from math import sqrt
import numpy as np
import image_getter as ig

PATH_COLOR = (100, 100, 0)
PATH_THICKNESS = 2
PATH_CIRC = 1


def pointsToImage(image, points, coef):
	"""Iscrta tocke na sliku."""
	
	for p in points:
		x, y = p.location.x, p.location.y
		x, y = int(x / coef), int(y / coef)
		cv2.circle(image, (x, y), PATH_CIRC, PATH_COLOR, PATH_THICKNESS)


def tuplesToImage(image, points, coef):
	"""Iscrta tocke na sliku (tocke su parovi)."""

	for (x, y) in points:
		x, y = int(x / coef), int(y / coef)
		cv2.circle(image, (x, y), PATH_CIRC, PATH_COLOR, PATH_THICKNESS)
	

class Image:
	"""Razred slike, olaksava koristenje OpenCV-a."""
	
	OPTIMAL_SIZE = 15000.0		# optimalna velicina slike (sirina x visina)
	CENTERDET = 25 			# konstanta za krugove (sto manja, vise ih nadje)
						
	
	def __init__(self, image, grayness=False):
		"""Konstruktor za objekt Image."""
		
		self._image = image				# ovo je OpenCV image dobiven iz imread
		self._grayness = grayness		# true ako je slika siva, false inace
		(self._width, self._height) = cv.GetSize(cv.fromarray(self._image))		# dimenzije
		self._coef = 1.0				# koeficijent resizanja slike [0..1]
		

	def getDimensions(self):
		"""
		Getter za dimenzije slike.
		Vraca (sirina, visina)
		"""
		
		return (self._width, self._height)
	
	
	def getImage(self):
		"""Getter za sliku."""
		
		return self._image
	
	
	def getCoef(self):
		"""Getter za koeficijent resizanja."""
		
		return self._coef
	
	
	def isGray(self):
		"""Vraca vrijednost grayness koja je true ako je slika siva."""
		
		return self._grayness
	
	
	def optimalResize(self):
		"""
		Mijenja dimenziju slike na "optimalnu velicinu".
		Uzme se tako da je veca dimenzija slike neka vrijednost izmedju 200 i 300.
		To se moze promijenit, ali u principu se dobije velicina koju
		A* algoritam brzo racuna.
		"""
		
		coef = self._calculateOptimalCoef()
		return self._resize(coef)
	
	
	def _resize(self, coef):
		"""Mijenja dimenziju slike koristeci dani koeficijent."""
		
		self._width = int(self._width * coef)
		self._height = int(self._height * coef)
		self._coef = coef
		
		self._image = cv2.resize(self._image, (self._width, self._height))
		return self
		
	
	def _calculateOptimalCoef(self):
		"""Racuna koeficijent za optimalnu velicinu slike."""
		
		maxTemp = self._width * self._height
		coef = sqrt(self.OPTIMAL_SIZE / maxTemp)
		return coef
	
	
	def _findCircles(self):
		"""Pronalazi sve krugove na slici."""
		
		return ig.NumpyImage(self._image).findCircles(self.CENTERDET)


	def _findTriangles(self):
		"""Pronalazi trokute na slici."""

		return ig.NumpyImage(self._image).findPolygon(3)
		
		
	def findStartCoef(self):
		"""
		Od svih pronadjenih krugova pokusa naci onaj koji je zelene boje.
		Vraca koeficijente x i y koordinata, odnosno vrijednosti
		koordinate x podijeljene sa sirinom i koordinate y podijeljene sa visinom.
		Ako nema takvoga vraca None.
		"""
		
		"""
		if self._grayness == True:
			raise StandardError("Za pronalazenje starta, slika mora biti u boji!")

		triangles = self._findTriangles()
		cts = ig.NumpyImage(self._image).filterColoredTriangles(triangles, ig.RED1)
		cts += ig.NumpyImage(self._image).filterColoredTriangles(triangles, ig.RED2)
		if len(cts) == 0:
			return None

		x, y = cts[0].findCenter()
		x, y = x + self._width / 2, self._height / 2 - y
		return (float(x) / self._width, float(y) / self._height)
		"""

		hsv_img = cv2.cvtColor(self._image, cv2.COLOR_BGR2HSV)
		cmin = ig.RED1_MIN
		cmax = ig.RED1_MAX
		frame_threshed = cv2.inRange(hsv_img, cmin, cmax)
		triangles = ig.NumpyImage(self._image).findPolygon(3)
		
		cmin = ig.RED2_MIN
		cmax = ig.RED2_MAX
		frame_threshed = cv2.inRange(hsv_img, cmin, cmax)
		triangles += ig.NumpyImage(self._image).findPolygon(3)
		
		if len(triangles) == 0:
			return None
		
		x, y = triangles[0].findCenter()
		x, y = x + self._width / 2, self._height / 2 - y
		return (float(x) / self._width, float(y) / self._height)


	def findEndCoef(self):
		"""
		Od svih krugova pokusa naci onaj koji je crvene boje.
		Vraca koeficijente x i y koordinata, odnosno vrijednosti
		koordinate x podijeljene sa sirinom i koordinate y podijeljene sa visinom.
		Ako takvog nema vraca None.
		"""

		if self._grayness == True:
			raise StandardError("Za pronalazenje starta, slika mora biti u boji!")
		
		circles = self._findCircles()
		ccs = ig.NumpyImage(self._image).filterColoredPolygons(circles, ig.GREEN)
		if len(ccs) == 0:
			return None
		
		x, y = ccs[0].getPosition()
		x, y = x + self._width / 2, self._height / 2 - y
		return (float(x) / self._width, float(y) / self._height)
					
				
	def convertToGray(self):
		"""
		Pretvara obojenu sliku u sivu.
		Vraca novu instancu ove klase koja ima sliku sive boje.
		"""
		
		if self._grayness == True:
			raise StandardError("Slika je vec siva!")
		grayImage = cv2.cvtColor(self._image, cv2.COLOR_BGR2GRAY) 
		return Image(grayImage, grayness=True)
	
	

class MapHandler:
	"""Klasa oponasa mapu labirinta za lakse izvodjenje A* algoritma."""
	
	BLACK = -1					# AStar ignorira -1 (zidovi)
	WHITE = 10					# radi boljeg puta je veci broj
	GRAYS = 220					# intenzitet sjene oko zidova, najveca vrijednost pa se smanjuje
	TRESHOLD = 60				# granica po kojoj se razlikuje bijelo od crnoga (0 = crno, 255 = bijelo)
	SHADE_THICKNESS = 6			# broj slojeva sjene oko zida
	GRAY_DIFF = 30				# razlika u nijansama sive ;)
	
	def __init__(self, grayImage):
		"""
		Konstruktor za map handler. Cijeli razred se svodi na Image
		razred ovog modula.
		"""
		
		if grayImage.isGray() == False:
			raise StandardError("Potrebna je siva slika.")
		self._grayImage = grayImage			# Image iz ovog modula
		self._grayList = self._calculateGrayList()
		
		
	def getGrayList(self):
		"""Getter za "sivu" listu."""
		
		return self._grayList
	
	
	def getDimensions(self):
		"""Dohvaca tuple za dimenzie mape (sirina, visina)."""
	
		return self._grayImage.getDimensions()
		
		
	def _calculateGrayList(self):
		"""Iz sive slike napravi listu koja reprezentira mapu labirinta."""
		
		li = []
		ba = bytearray(self._grayImage.getImage())
		for byte in ba:
			if byte < self.TRESHOLD:
				li += [self.BLACK]
			else:
				li += [self.WHITE]
			
		return li
	
	
	def wallShades(self):
		"""Napravi sjenu oko zidova da A* ode po sredini labirinta."""
		
		currentEdge = self.BLACK		# ove 2 varijable predstavljaju intenzitete sjene
		newEdge = self.GRAYS
		for _ in xrange(self.SHADE_THICKNESS):

			(w, h) = self.getDimensions()
			for x in xrange(w):
				for y in xrange(h):
					if self._grayList[y * w + x] == currentEdge:
						self._shadeIt(w, h, x, y, newEdge)
						
			currentEdge = newEdge
			newEdge -= self.GRAY_DIFF
					
				
	def _shadeIt(self, width, height, x, y, shadeIntensity):
		"""Za danu tocku napravi sjenu oko nje ako je trenutno nema."""
		
		points = [(x-1, y-1), (x-1, y+1), (x+1, y-1), (x+1, y+1)]
		for p in points:
			if p[0] >= 0 and p[1] >= 0 and p[0] < width and p[1] < height:
				pos = p[1] * width + p[0]
				if self._grayList[pos] == self.WHITE:
					self._grayList[pos] = shadeIntensity




class PathFinder:
	"""Ova klasa sluzi za trazenje puta kroz labirint."""
	
	
	def __init__(self, mapHandler):
		"""Konstruktor prima map handler pomocu kojega trazi put kroz labirint."""
		
		self._mh = mapHandler		# map handler iz ovog modula
	
	
	def findPath(self, start, end):
		"""
		Vrti A* algoritam i pronalazi put kroz labirint.
		Vraca listu cvorova.
		"""
		
		array = self._mh.getGrayList()
		(width, height) = self._mh.getDimensions()
		mh = AStar.SQ_MapHandler(array, width, height)
		
		(xs, ys) = (start[0], start[1])
		start = AStar.SQ_Location(xs, ys)
		(xe, ye) = (end[0], end[1])
		end = AStar.SQ_Location(xe, ye)
		
		astar = AStar.AStar(mh)
		path = astar.findPath(start, end)
		if path == None:		# nema puta
			return None
		
		nodes = path.getNodes()
		return nodes
	
					