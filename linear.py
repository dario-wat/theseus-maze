import math
from abc import ABCMeta, abstractmethod


def _distance((x1, y1), (x2, y2)):
	"""Racuna udaljenost 2 tocke u ravnini."""

	return math.sqrt((x2-x1)**2 + (y2-y1)**2)


class AbstractPolygon:
	"""Apstraktan razred za n-terokute."""

	__metaclass__ = ABCMeta
	
	@abstractmethod
	def getContours(self):
		pass
		
	@abstractmethod
	def getVertices(self):
		pass
		
	@abstractmethod
	def findCenter(self):
		pass	


class Vector:
	"""Klasa za vektore."""

	def __init__(self, (x, y)):
		"""Postavljam vektor. (Druga tocka je uvijek ishodiste)"""
		
		self.x, self.y = x, y
		
	
	def normalize(self):
		"""Normalizira vektor."""
		
		n = self.norm()
		self.x /= n
		self.y /= n
		return self


	def angle(self):
		"""Pronadje kut -pi do pi."""

		return -(math.atan2(self.y, self.x) - math.pi / 2)


	def angle2(self):
		"""Pronadje kut od 0 do 2pi. (nije to sto pise)"""

		return math.atan2(self.y, self.x)


	@staticmethod
	def vectorFromAngle(angle):
		"""Nadje jedinicni vektor iz kuta. Kut moze biti 0 do 2pi."""

		x = 1
		y = math.tan(angle)
		if abs(angle) > math.pi/2 and abs(angle) < math.pi:
			x = -x
			y = -y 
		return Vector((x, y)).normalize()


	def getTuple(self):
		"""Vrati tuple."""

		return self.x, self.y
		
		
	def norm(self):
		"""Racuna normu vektora."""
		
		return math.sqrt(self.x * self.x + self.y * self.y)
		
		
	def __repr__(self):
		"""Overridam metodu za printanje."""
		
		return "(%f, %f)" % (self.x, self.y)
		
		
class Triangle(AbstractPolygon):
	"""Razred za trokute koje pronadje na slici."""
	
	EPSILON = 10	# greska racuna slike (promasaj izmedju 2 jednake stranice)
	
	def __init__(self, contours, vertices):
		"""Stvori trokut od kontura i vrhova."""
		
		if len(vertices) != 3:
			raise StandardError("Nije trokut")
			
		self.a = vertices[0]
		self.b = vertices[1]
		self.c = vertices[2]
		
		# nemaju translatirane koordinate u sluze samo za crtanje
		self._contours = contours
		
		
	def getContours(self):
		"""Getter za konture trokuta."""
		
		return self._contours
		
	
	def getVertices(self):
		"""Dohvaca vrhove trokuta. Vrati listu tupla."""
		
		return [self.a, self.b, self.c]
		
		
	def findCenter(self):
		"""Pronalazi sredinu trokuta (teziste)."""
	
		x1, y1 = self.a
		x2, y2 = self.b
		x3, y3 = self.c
		x = float(x1 + x2 + x3) / 3
		y = float(y1 + y2 + y3) / 3
		return (int(x), int(y))
	
	
	def findDirection(self):
		"""
		Racuna smjer trokuta odnosno vektor koji povezuje poloviste osnovice
		jednakokracnog trokuta i nasuprotan vrh. (visina)
		"""
	
		d1 = _distance(self.a, self.b)		# udaljenost tocke 1 i 2
		d2 = _distance(self.a, self.c)		# udaljenost tocke 1 i 3
		d3 = _distance(self.b, self.c)		# udaljenost tocke 2 i 3
	
		# odredjivanje osnovice i krakova, tj. tocaka koje su na osnovici
		# i na suprotnom vrhu. p1 i p2 su uz osnovicu, v je suprotan vrh
		p1 = p2 = v = None
		if abs(d3-d2) < self.EPSILON:
			p1 = self.a
			p2 = self.b
			v = self.c
		elif abs(d2-d1) < self.EPSILON:
			p1 = self.b
			p2 = self.c
			v = self.a
		else:
			p1 = self.a
			p2 = self.c
			v = self.b
		
		halfPoint = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
		vec = (v[0] - halfPoint[0], v[1] - halfPoint[1])
		return Vector(vec).normalize()
		
			
	def __repr__(self):
		"""Overridam ispis."""
		
		return str(self.a) + " " + str(self.b) + " " + str(self.c)


class Circle:
	"""Razred za krugove koje pronadje na slici."""

	POINT_DIFF = 3		# threshold za odredjivanje tocke na kruznici
	RADIUS_DIFF = 3		# threshold za razliku radius i d (d=a+b)

	def __init__(self, radius, (x, y)):
		"""
		Konstruktor za krugove. Prima radius kruga i koordinate sredista.
		"""
		
		self.radius = radius
		self.x, self.y = x, y
		
		
	def getPosition(self):
		"""Vraca koordinate pozicije sredista kruznice."""
		
		return self.x, self.y


	def intersect(self, other):
		"""
		Pronadje sjecista ove kruznice i dane kruznice.
		Vraca None ako nema sjecista.
		"""

		d = _distance((self.x, self.y), (other.x, other.y))
		if d > self.radius + other.radius:
			return None

		a = (self.radius ** 2 - other.radius ** 2 + d ** 2) / (2*d)
		k = self.radius ** 2 - a ** 2
		if k < 0:
			return None

		h = math.sqrt(k)
		x2, y2 = self.x + a*(other.x-self.x)/d, self.y + a*(other.y-self.y)/d

		x31, x32 = x2 + h*(other.y-self.y)/d, x2 - h*(other.y-self.y)/d
		y31, y32 = y2 - h*(other.x-self.x)/d, y2 + h*(other.x-self.x)/d
		return [(x31, y31), (x32, y32)]


	def contains(self, (x, y)):
		"""Funkcija provjerava, sadrzi li kruznica neku tocku."""

		return (x-self.x)**2 + (y-self.y)**2 - self.radius**2 < self.POINT_DIFF
		
		
	def __repr__(self):
		"""Overridam ispis."""
		
		return "(%d, %d) %d" % (self.x, self.y, self.radius)
		
		
class Rectangle(AbstractPolygon):
	"""Razred za pravokutnike."""
	
	def __init__(self, contours, vertices):
		"""Konstruktor za pravokutnike."""
		
		if len(vertices) != 4:
			raise StandardError("Nije pravokutnik")
			
		self.a = vertices[0]
		self.b = vertices[1]
		self.c = vertices[2]
		self.d = vertices[3]
		
		# nemaju translatirane koordinate i sluze samo za crtanje
		self._contours = contours
		
		
	def getVertices(self):
		"""Dohvat vrhova pravokutnika."""
		
		return [self.a, self.b, self.c, self.d]
		
		
	def getContours(self):
		"""Dohvat kontura."""
		
		return self._contours
		
		
	def findCenter(self):
		"""Pronadje srediste pravokutnika."""
		
		a, b, c, d = self._resolveAdjacentVertices()
		line1 = Line(a, c)
		line2 = Line(b, d)
		center = line1.intersection(line2)
		return center

			
	def _resolveAdjacentVertices(self):
		"""
		Odredi koj su vrhovi susjedni. Vraca redom 2 i 2
		susjedna vrha u pravokutniku.
		"""
		
		d1 = _distance(self.a, self.b)		# udaljenost tocke 1 i 2
		d2 = _distance(self.a, self.c)		# udaljenost tocke 1 i 3
		d3 = _distance(self.a, self.d)		# udaljenost tocke 1 i 4
		
		# pronalazi stranice odnosno tocke koje su susjedne
		# a i c, b i d su na dijagonalama
		a = self.a
		b = c = d = None
		if d1 > d2 and d1 > d3:
			c = self.b
			b = self.c
			d = self.d
		elif d2 > d3:
			c = self.c
			b = self.b
			d = self.d
		else:
			c = self.d
			b = self.b
			d = self.c
			
		return a, b, c, d
		
		
	def __repr__(self):
		"""Overridam ispis."""
		
		return str(self.getVertices())
		
		
class Line:
	"""Razred za pravac."""
	
	EPSILON = 0.1			# greska pri racunu razlike kuta
	INF = float("inf")		# infinity
	
	def __init__(self, pointA, pointB):
		"""Konstruktor za pravac kroz 2 tocke."""
		
		self.a = pointA
		self.b = pointB
		
		
	def angle(self):
		"""Racuna nagib pravca."""
		
		x1, y1 = self.a
		x2, y2 = self.b
		if abs(x2 - x1) == 0:
			return self.INF
		return float(y2 - y1) / (x2 - x1)


	def angle2(self):
		"""Racuna kuta izmedju -pi/2 i pi/2 za nagib. 0 je paralelan s osi y."""

		x1, y1 = self.a
		x2, y2 = self.b
		x = x2 - x1
		y = y2 - y1

		c1 = math.atan(float(y)/x) + math.pi/2
		c2 = math.atan(float(y)/x) - math.pi/2
		if abs(c1) < abs(c2):
			return -c1
		return -c2


	def vector(self):
		"""Nadje vektor za pravac."""

		x1, y1 = self.a
		x2, y2 = self.b
		#return Vector( ((x1 - x2), (y1 - y2)) ) if x2 - x1 < 0
		return Vector( ((x2 - x1), (y2 - y1)) )


	def length(self):
		"""Racuna duljinu linije."""

		return _distance(self.a, self.b)
	
	
	def intersection(self, other):
		"""Nadje sjeciste izmedju ovog i jos jednog pravca."""
		
		# zbog nepreciznosti aproksimacije vrhova se koristi ova implementacija
		return self._alternativeIntersection(other)
		
		"""
		k1 = self.angle()
		k2 = other.angle()
		if k1 == self.INF or k2 == self.INF or abs(k1 - k2) < self.EPSILON:
			return self._alternativeIntersection(other)
		
		x1, y1 = self.a
		x2, y2 = other.a
		
		x = int((y1 - y2 + x2 * k2 - x1 * k1) / (k2 - k1))
		y = int(y1 + k1 * (x - x1))
		return x, y
		"""
		
		
	def _alternativeIntersection(self, other):
		"""Racuna aproksimaciju sjecista pomocu polovista."""
		
		hx1, hy1 = self.halfPoint()
		hx2, hy2 = other.halfPoint()
		x = (hx1 + hx2) / 2
		y = (hy1 + hy2) / 2
		return x, y
		
		
	def halfPoint(self):
		"""Racuna poloviste izmedju koordinata pomocu kojih je pravac zadan."""
		
		x1, y1 = self.a
		x2, y2 = self.b
		return (x1 + x2) / 2, (y1 + y2) / 2


	def __repr__(self):
		"""Ispis linije."""

		return str(self.a) + ', ' + str(self.b)
