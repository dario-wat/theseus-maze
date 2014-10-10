from math import sqrt, cos, sin, pi, tan
from linear import Circle, Vector
from operator import itemgetter
import roslib; roslib.load_manifest('ardrone_autonomy')
import image_getter as ig
import cv2
import rospy
from geometry_msgs.msg import Pose
from ardrone_autonomy.msg import Navdata
from filterlib import Tocke, Filter

SF_LENGTH = 225		# udaljenost ulaza i izlaza u centimetrima

# dictionary sa vrijednostima pozicija svih markera
# sve vrijednosti su u centimetrima
# i sve je dobiveno eksperimentalno
GLOBAL_LOC = 	{	'yellow': (-62, -60), 'blue': (56, -99),
					'orange': (-61, -153), 'purple': (58, -183),
					'exit': (0, 0)
				}

DIAGONAL = 200		# dijagonala kamere
ANGLE_CAM = 46		# kut kamere
EXIT_CONST = 20 	# za trazenje izlaza (zeleni krug)
FILTER_Q_TRESHOLD_C = 50
FILTER_Q_TRESHOLD_D = 71
FILTER_T_TRESHOLD_D = 20
FILTER_T_TRESHOLD_V = 0.8

M_EXIT = 'exit'
M_BLUE = 'blue'
M_YELLOW = 'yellow'
M_ORANGE = 'orange'
M_PURPLE = 'purple'

_prevQuad = 0, 0		# za filter, za poziciju quada
_prevTezejPos = None		# za filter, za tezeja (za poziciju)
_prevTezejAngle = None		# za smjer


class Locator:

	def __init__(self, (x, y), angle, pixelSize):
		self.x = x
		self.y = y
		self.angle = angle 		# negativan desno od y
		self.pixelSize = pixelSize


	def findLocation(self, (x, y)):
		"""
		Pronadje globalnu lokaciju tocke u labirintu.
		Tocka je relativna sa slike.
		"""

		x, y = _rotateCoordinates((x, y), self.angle)
		x, y = _prepareCoordinates((x, y), self.pixelSize)
		return (int(self.x + x), int(self.y + y))


	def findDirection(self, (x, y)):
		"""Funkcija nadje vektor smjera. (x, y) je vektor. Vraca jedinicni."""

		angle = Vector((x, y)).angle2() + self.angle
		return Vector.vectorFromAngle(angle)


class ImageTagger:
	"""Klasa za oznacavanje slike."""

	def __init__(self, image, tagFlag):
		self._image = image
		self._tagFlag = tagFlag 	# govori ocu li oznacavat sliku


	def markCircles(self, circles):
		"""Oznaci krugove."""

		if self._tagFlag:
			self._image.markCircles(circles)

	def markPolygons(self, polygons):
		"""Oznaci n-terokute."""

		if self._tagFlag:
			self._image.markPolygons(polygons)

	def markLines(self, lines):
		"""Oznaci linije."""

		if self._tagFlag:
			self._image.markLines(lines)

	def getNumpyArray(self):
		"""Vrati numpy array."""

		return self._image.getNumpyArray()


def _calculatePixelSize(pixelDist, realDist):
	"""Racuna velicinu piksela u centimetrima."""
	
	return float(realDist) / pixelDist


#deprecated
def _pixel2centimeter((x, y), height):
	"""Racuna udaljenost u centimetrima."""

	alphax = float(x) / DIAGONAL * ANGLE_CAM
	alphay = float(y) / DIAGONAL * ANGLE_CAM
	xc = float(height) / 10 * tan((alphax/180) * pi)
	yc = float(height) / 10 * tan((alphay/180) * pi)
	return (xc, yc)


def tuplesPath(path):
	"""Pretvori path iz liste cvorova u listu tupla."""

	return map(lambda n: (n.location.x, n.location.y), path)


def translatePath(path, start, end):
	"""
	Translatira koordinate sa slike tako da su koordinate
	izlaza == (0,0). Te translatirane koordinate pretvori
	u globalne koordinate labirinta. Vraca listu tupla.
	path - lista tupla
	"""

	xe, ye = end
	xs, ys = start
	pathTr = map(lambda (x, y): (x - xe, y - ye), path)

	coef = _calculatePixelSize(sqrt((xe-xs)**2 + (ye-ys)**2), SF_LENGTH)
	return map(lambda (x, y): (int(coef*x), int(coef*y)), pathTr)


def locateQuad(markers, angle):
	"""
	Pronalazi lokaciju quada u globalnom koordinatnom sustavu na temelju
	markera koje dobije iz dictionarija. U dictionariju se nalaze pixelne
	koordinate markera od trenutne tocke. Za pronalazak pozicije koristi
	2 tocke, odnosno 2 markera.
	"""

	legalMarkers = _legalMarkers(markers)
	if len(legalMarkers) < 2:
		return None
	legalMarkers = _chooseOptimal(legalMarkers)
	v = legalMarkers.values()

	# globalne koordinate 2 markera
	x1, y1 = GLOBAL_LOC[legalMarkers.keys()[0]]
	x2, y2 = GLOBAL_LOC[legalMarkers.keys()[1]]
	
	# racunanje koeficijenta za dimenzioniranje koordinatnog sustava
	# krajnja vrijednost je [cm/pixel]
	localMarkerDist = _distance(v[0], v[1])
	globalMarkerDist = _distance((x1, y1), (x2, y2))
	pixelSize = _calculatePixelSize(localMarkerDist, globalMarkerDist)
	
	if angle is None:	# losa aproksimacija kuta, ovo nebi trebalo cesto bit
		angle = 0
	v = _prepareCoordsList(v, pixelSize)
	v = _rotateCoordsList(v, angle)

	# rotirane koordinate u centimetrima koje daju udaljenost do markera
	x1r, y1r = v[0]
	x2r, y2r = v[1]

	point1 = (int(x1-x1r), int(y1-y1r))
	point2 = (int(x2-x2r), int(y2-y2r))
	x, y = (point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2
	fp = _filterDQuad((x, y))
	return None if fp is None else Locator(fp, angle, pixelSize)


def _filterC((x, y)):
	"""Filtrira tocku u odnosu na koordinate."""

	global _prevQuad
	xp, yp = _prevQuad
	if abs(x-xp) > FILTER_Q_TRESHOLD_C or abs(y-yp) > FILTER_Q_TRESHOLD_C:
		return None
	else:
		_prevQuad = x, y
		return x, y


def _filterDQuad((x, y)):
	"""Filtrira tocku u odnosu na udaljenost."""

	global _prevQuad
	xp, yp = _prevQuad
	d = _distance((x, y), (xp, yp))
	if d > FILTER_Q_TRESHOLD_D:
		return None
	else:
		_prevQuad = x, y
		return x, y


def _distance((x1, y1), (x2, y2)=(0, 0)):
	"""Udaljenost dvije tocke."""

	return sqrt((x2-x1)**2 + (y2-y1)**2)


def _legalMarkers(markers):
	"""
	Filtrira sve markere koje je pronaso. Vrati dict markera.
	Ukloni one kojima je vrijednost None.
	"""

	return {k:v for k, v in markers.items() if v is not None}


def _prepareCoordsList(coords, coef):
	"""Pretvori listu koordinata u centimetre."""

	return [_prepareCoordinates(c, coef) for c in coords]


def _prepareCoordinates((x, y), coef):
	"""Pretvori pixelne koordinate u centimetre. Coef je [cm/pixel]"""

	return x*coef, y*coef


def _chooseOptimal(markers):
	"""
	Odabere najbolja 2 markera za trazenje tocaka. To bi bila dva
	markera koja su najbliza trenutnoj poziciji.
	U trenutku kad se ova funkcija pozove, u njoj su sigurno najmanje
	2 markera.
	"""

	distMarkers = map(lambda (k, v): (k, v, _distance(v)), markers.items())
	sortedMarkers = sorted(distMarkers, key=itemgetter(2))
	return {sortedMarkers[0][0]: sortedMarkers[0][1],
		sortedMarkers[1][0]: sortedMarkers[1][1]}


def _rotateCoordsList(coords, theta):
	"""Rotira listu koordinata za dani kut."""

	return map(lambda c: _rotateCoordinates(c, theta), coords)


def _rotateCoordinates((x,y), theta):
	"""Rotira koordinate (x,y) za kut theta."""

	xr = x*cos(theta) - y*sin(theta)
	yr = x*sin(theta) + y*cos(theta)
	return (xr, yr)


#deprecated
def _locateQuad(markers, angle, height):
	"""
	Pronadje poziciju quad-a u globalnom sustavu.
	markers - dictionary sa kljucevima: yellow, blue, purple, orange i exit
	vrijednosti su udaljenost koordinata od trenutnog polozaja tezeja
	angle - kut otklona pogleda letjelice (na lijevo mora bit negativan)
	"""

	legalMarkers = _legalMarkers(markers)
	markerCount = len(legalMarkers)

	if markerCount == 0:
		return None
	
	# ne koristim
	if markerCount == 1:
		return None			# trebao bi preko 2 tocke naci poziciju
		if angle is None:	# losa aproksimacija kuta
			angle = 0
		x, y = _pixel2centimeter(legalMarkers.values()[0], height)
		xr, yr = _rotateCoordinates((x, y), angle)
		xd, yd = GLOBAL_LOC[legalMarkers.keys()[0]]
		#print (int(xd - xr), int(yd - yr))
		return (int(xd - xr), int(yd - yr))

	if markerCount >= 2:
		v = legalMarkers.values()
		x1, y1 = GLOBAL_LOC[legalMarkers.keys()[0]]
		x2, y2 = GLOBAL_LOC[legalMarkers.keys()[1]]
		localMarkerDist = _distance(v[0], v[1])
		globalMarkerDist = _distance((x1, y1), (x2, y2))
		pixelSize = _calculatePixelSize(localMarkerDist, globalMarkerDist)
		v = _prepareCoordsList(v, pixelSize)

		d1 = _distance(v[0])
		d2 = _distance(v[1])
		points = Circle(d1, (x1, y1)).intersect(Circle(d2, (x2, y2)))
		if points == None:
			return None
		#print "(%.2f %.2f) (%.2f %.2f)" % (points[0][0], points[0][1],
		#	points[1][0], points[1][1])
		# ne bas dobra aproksimacija odabira tocke
		if v[0][1] < 0:
			return points[0] if points[0][1] > 0 else points[1]
		return points[0] if points[0][1] < 0 else points[1]
		
	# ne koristim
	if markerCount >= 3:
		return None
		v = legalMarkers.values()
		d1 = _distance(v[0])
		d2 = _distance(v[1])
		d3 = _distance(v[2])
		x1, y1 = GLOBAL_LOC[legalMarkers.keys()[0]]
		x2, y2 = GLOBAL_LOC[legalMarkers.keys()[1]]
		x3, y3 = GLOBAL_LOC[legalMarkers.keys()[2]]
		points = Circle(d1, (x1, y1)).intersect(Circle(d2, (x2, y2)))
		if points == None:
			return None
		if Circle(d3, (x3, y3)).contains(points[0]):
			return points[0]
		return points[1]


#deprecated
def _cornerPath(path):
	"""
	Iz tocaka puta izvuce samo tocke u kojima treba promijeniti smjer.
	path - lista cvorova
	"""
	
	diff = (0, 0)
	cpath = []
	for i in xrange(len(path) - 1):
		d = (path[i+1].location.x - path[i].location.x,
			path[i+1].location.y - path[i].location.y)
		if d != diff:
			diff = d
			cpath.append(path[i])
			
	cpath.append(path[len(path) - 1])
	return cpath


def _findMarkers(npimg, data, imgTagger):
	"""Pronadje markere na slici i vrati dictionary."""

	markers = {}
	_findExit(npimg, data, markers, imgTagger)
	_findRectangularMarkers(npimg, data, markers, imgTagger)
	return markers


def locateYourself(npimg, data, imgTagger):
	"""
	Pronadje svoju poziciju pomocu slike. Vraca globalne koordinate
	polozaja (x, y). Funkcija publisha sve pronadjene markere.
	"""

	markers = _findMarkers(npimg, data, imgTagger)
	angle = _orientateYourself(npimg, data, imgTagger)
	return locateQuad(markers, angle)


def _findExit(npimg, data, markers, imgTagger):
	"""Trazi marker koji oznacava izlaz."""

	circles = npimg.findCircles(EXIT_CONST)
	greenCircles = npimg.filterColoredCircles(circles, ig.GREEN)
	if greenCircles is not None and len(greenCircles) > 0:
		circ = greenCircles[0]
		imgTagger.markCircles([circ])
		center = circ.getPosition()
		data.exitMarker.position.x = center[0]
		data.exitMarker.position.y = center[1]
		data.pubPoseExit.publish(data.exitMarker)
		markers[M_EXIT] = center


def _findRectangularMarkers(npimg, data, markers, imgTagger):
	"""Pronadje sve pravokutne markere."""

	# ovo su imena i vrijednosti markera
	values = [	(M_PURPLE, ig.PURPLE, data.purpleMarker, data.pubPosePM),
				(M_ORANGE, ig.ORANGE, data.orangeMarker, data.pubPoseOM),
				(M_YELLOW, ig.YELLOW, data.yellowMarker, data.pubPoseYM),
				(M_BLUE, ig.BLUE, data.blueMarker, data.pubPoseBM)
			]

	rectangles = npimg.findPolygon(4)
	if rectangles is not None and len(rectangles) > 0:
		for value in values:
			_findRectangular(npimg, markers, value, rectangles, imgTagger)


def _findRectangular(
	npimg, markers, (name, color, pose, publisher), rectangle, imgTagger):
	"""Pronadje marker na slici s odredjenom bojom i danim imenom."""

	coloredRects = npimg.filterColoredPolygons(rectangle, color)
	if coloredRects is not None and len(coloredRects) > 0:
		rect = coloredRects[0]
		imgTagger.markPolygons([rect])
		center = rect.findCenter()
		pose.position.x = center[0]
		pose.position.y = center[1]
		publisher.publish(pose)
		markers[name] = center


def _orientateYourself(npimg, data, imgTagger):
	"""Orijentira se u labirintu."""

	lines = npimg.findLines()
	if lines is not None:
		# algoritam trazenja najbolje linije za orjentaciju
		minAng = min(map(lambda l: abs(l.angle2()), lines))
		line = filter(lambda l: abs(minAng - abs(l.angle2())) < 0.1, lines)
		lineLen = map(lambda l: (l.length(), l), line)
		lineLen = sorted(lineLen, reverse=True)
		line = [lineLen[0][1]]
		
		imgTagger.markLines(line)
		direction = line[0].angle2()
		data.north.orientation.z = direction
		data.pubPoseNorth.publish(data.north)
		return direction
	return None

filtl=Filter(0.7)
filtd=Filter(0.7)
ltoc=Tocke()
nltoc=Tocke()
dtoc=Tocke()
ndtoc=Tocke()

def locateTezej(npimg, data, locator, imgTagger):
	"""
	Pronadje globalne koordinate tezeja (crveni trokut).
	Vraca (lokacija, smjer).
	"""

	"""
	triangles = npimg.findPolygon(3)
	coloredTriangles = npimg.filterColoredPolygons(triangles, ig.RED1)
	coloredTriangles += npimg.filterColoredPolygons(triangles, ig.RED2)	
	
	if coloredTriangles is not None and len(coloredTriangles) > 0:
		triangle = coloredTriangles[0]
		imgTagger.markPolygons([triangle])
		center = triangle.findCenter()
		
		data.position.position.x = center[0]
		data.position.position.y = center[1]
		data.pubPoseTrokut.publish(data.position)
		
		location = locator.findLocation(center)
		d = triangle.findDirection()
		direction = locator.findDirection((d.x, d.y))
		return _filterTezejPos(location), _filterTezejAngle(direction.getTuple())
	return None, None
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
		return None, None
	
	triangle = triangles[0]
	imgTagger.markPolygons([triangle])
	center = triangle.findCenter()

	# publishanje
	data.position.position.x = center[0]
	data.position.position.y = center[1]
	data.pubPoseTrokut.publish(data.position)

	# izracun pozicije i smjera
	location = locator.findLocation(center)
	d = triangle.findDirection()
	direction = locator.findDirection((d.x, d.y)).getTuple()
	
	# kroz filtere
	ltoc.x, ltoc.y = location
	nltoc = filterl.lowpass(ltoc)
	location = nltoc.x, nltoc.y

	dtoc.x, dtoc.y = direction
	ndtoc = filterd.lowpass(dtoc)
	direction = ndtoc.x, ndtoc.y

	return location, direction


def _filterTezejPos((x, y)):
	"""Filtrira poziciju za tezeja."""

	global _prevTezejPos
	if _prevTezejPos is None:
		_prevTezejPos = (x, y)
		return _prevTezejPos

	xp, yp = _prevTezejPos
	d = _distance((x, y), (xp, yp))
	if d > FILTER_T_TRESHOLD_D:
		return None
	else:
		_prevTezejPos = x, y
		return x, y
		

def _filterTezejAngle((x, y)):
	"""Filtrira kut tezeja."""

	global _prevTezejAngle
	if _prevTezejAngle is None:
		_prevTezejAngle = (x, y)
		return _prevTezejAngle

	xp, yp = _prevTezejAngle
	print "%.2f %.2f %.2f %.2f" % (x, y, xp, yp)
	if abs(xp-x) > FILTER_T_TRESHOLD_V or abs(yp-y) > FILTER_T_TRESHOLD_V:
		return None
	else:
		_prevTezejAngle = x, y
		return x, y


def locateMinotaurs(npimg, locator):
	"""
	Pronadje globalne koordinate i smjerove minotaura na slici.
	Vraca listu minotaura [(lokacija, smjer)].
	"""

	triangles = npimg.findPolygon(3)
	coloredTriangles = npimg.filterColoredPolygons(triangles, ig.GREEN)
	if coloredTriangles is None:
		return []

	def _mapM(triangle):
		"""Pomocna za mapiranje trokuta."""

		location = locator.findLocation(triangle.findCenter())
		d = triangle.findDirection()
		direction = locator.findDirection((d.x, d.y))
		return (location, direction)

	return map(_mapM, coloredTriangles)


class ROSData:

	def __init__(self):

		self.pubPoseTrokut = rospy.Publisher("/camera/tezej", Pose)
		self.position = Pose()
	
		self.pubPoseNorth = rospy.Publisher("/camera/north", Pose)
		self.north = Pose()

		self.pubPoseYM = rospy.Publisher("/camera/marker/yellow", Pose)
		self.yellowMarker = Pose()

		self.pubPoseOM = rospy.Publisher("/camera/marker/orange", Pose)
		self.orangeMarker = Pose()
	
		self.pubPoseBM = rospy.Publisher("/camera/marker/blue", Pose)
		self.blueMarker = Pose()
	
		self.pubPosePM = rospy.Publisher("/camera/marker/purple", Pose)
		self.purpleMarker = Pose()

		self.pubPoseExit = rospy.Publisher("/camera/exit", Pose)
		self.exitMarker = Pose()
		
		self.navdata = Navdata()
		#self.subNavdata = rospy.Subscriber("/ardrone/navdata", Navdata, self.setNav)

		self.pubPoseGlobal = rospy.Publisher("/globus", Pose)
		self.globalCoords = Pose()


	def setNav(self, navdata):
		self.navdata = navdata

	def getHeight(self):
		return self.navdata.altd
