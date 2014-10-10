#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone_autonomy')
import image_getter as ig
import camera_streamer as cs
from geometry_msgs.msg import Pose
from ardrone_autonomy.msg import Navdata
import rospy
import maze_solver as ms
import cv2

def callback(image):
	"""Pronalazi trokute odredjene boje."""
	
	global data

	rosImg = ig.RospyImage(image)
	npimg = rosImg.createNumpyImage()
	npimgCopy = rosImg.createNumpyImage()

	markers = {}
	angle = None
	
	triangles = npimg.findPolygon(3)
	coloredTriangles = npimg.filterColoredPolygons(triangles, ig.RED1)
	coloredTriangles += npimg.filterColoredPolygons(triangles, ig.RED2)	
	if coloredTriangles is not None and len(coloredTriangles) > 0:
		triangle = coloredTriangles[0]
		#npimgCopy.markPolygons([triangle])
		center = triangle.findCenter()
		data.position.position.x = center[0]
		data.position.position.y = center[1]
		data.pubPoseTrokut.publish(data.position)

	lines = npimg.findLines()
	if lines is not None:
		minAng = min(map(lambda l: abs(l.angle2()), lines))
		line = filter(lambda l: abs(minAng - abs(l.angle2())) < 0.1, lines)
		lineLen = map(lambda l: (l.length(), l), line)
		lineLen = sorted(lineLen, reverse=True)
		line = [lineLen[0][1]]
		npimgCopy.markLines(line)
		direction = line[0].angle2()
		angle = direction
		data.north.orientation.z = direction
		data.pubPoseNorth.publish(data.north)

	rectangles = npimg.findPolygon(4)
	if rectangles is not None and len(rectangles) > 0:
		yellow = npimg.filterColoredPolygons(rectangles, ig.YELLOW)
		orange = npimg.filterColoredPolygons(rectangles, ig.ORANGE)
		blue = npimg.filterColoredPolygons(rectangles, ig.BLUE)
		purple = npimg.filterColoredPolygons(rectangles, ig.PURPLE)

		if yellow is not None and len(yellow) > 0:
			rect = yellow[0]
			npimgCopy.markPolygons([rect])
			center = rect.findCenter()
			data.yellowMarker.position.x = center[0]
			data.yellowMarker.position.y = center[1]
			data.pubPoseYM.publish(data.yellowMarker)
			markers['yellow'] = center

		if orange is not None and len(orange) > 0:
			rect = orange[0]
			npimgCopy.markPolygons([rect])
			center = rect.findCenter()
			data.orangeMarker.position.x = center[0]
			data.orangeMarker.position.y = center[1]
			data.pubPoseOM.publish(data.orangeMarker)
			markers['orange'] = center
			
		if blue is not None and len(blue) > 0:
			rect = blue[0]
			npimgCopy.markPolygons([rect])
			center = rect.findCenter()
			data.blueMarker.position.x = center[0]
			data.blueMarker.position.y = center[1]
			data.pubPoseBM.publish(data.blueMarker)
			markers['blue'] = center
		
		if purple is not None and len(purple) > 0:
			rect = purple[0]
			npimgCopy.markPolygons([rect])
			center = rect.findCenter()
			data.purpleMarker.position.x = center[0]
			data.purpleMarker.position.y = center[1]
			data.pubPosePM.publish(data.purpleMarker)
			markers['purple'] = center
		

	circles = npimg.findCircles(20)
	greenCircles = npimg.filterColoredCircles(circles, ig.GREEN)
	if greenCircles is not None and len(greenCircles) > 0:
		circ = greenCircles[0]
		npimgCopy.markCircles([circ])
		center = circ.getPosition()
		data.exitMarker.position.x = center[0]
		data.exitMarker.position.y = center[1]
		data.pubPoseExit.publish(data.exitMarker)
		markers['exit'] = center

	"""
	blueCircles = npimg.filterColoredCircles(circles, ig.BLUE)
	if blueCircles is not None and len(blueCircles) > 0:
		circ = blueCircles[0]
		npimgCopy.markCircles([circ])
		center = circ.getPosition()
		data.purpleMarker.position.x = center[0]
		data.purpleMarker.position.y = center[0]
		data.pubPosePM.publish(data.purpleMarker)
		markers['purple'] = center
	"""

	loc = ms.locateQuad(markers, angle)
	if loc is not None:
		data.globalCoords.position.x = loc[0]
		data.globalCoords.position.y = loc[1]
		data.pubPoseGlobal.publish(data.globalCoords)
		print "%6.2f %6.2f" % (loc[0], loc[1])
	X = 320 / 2
	Y = 240 / 2
	cv2.circle(npimgCopy.getNumpyArray(), (X, Y), 2, (0,0,0), 2)
	cv2.imshow('stream', npimgCopy.getNumpyArray())
	cv2.waitKey(1)


def init():
	"""Inicijalizacija."""

	global data
	ig.init_node()
	rospy.Rate(10)
	data = ROSData()
	

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
		self.subNavdata = rospy.Subscriber("/ardrone/navdata", Navdata, self.setNav)

		self.pubPoseGlobal = rospy.Publisher("/globus", Pose)
		self.globalCoords = Pose()


	def setNav(self, navdata):
		self.navdata = navdata

	def getHeight(self):
		return self.navdata.altd



if __name__ == "__main__":
	
	init()
	#ig.newListener(callback)
	ig.newListener(cs.colorCallback)
	#ig.newListener(cs.rectangleCallback)
	#ig.newListener(cs.triangleCallback)
	#ig.newListener(cs.yellowHSV)
	ig.rospySpin()
	cs.destroyAllWindows()
