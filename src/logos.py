#!/usr/bin/env python

import rospy
import cv2
import socket
import image_getter as ig
import maze_solver as ms
import controlMoway as cm
import pathfinder as pf
from sensor_msgs.msg import Image
from lineToDots import saveCornerDots
import projektlib

RATE = 10
TCP_IP = '192.168.1.28'
TCP_PORT = 9760


class Stanje:
	"""Stanja automata."""

	pocetno = 1
	iteracija = 2


stanje = Stanje.pocetno
sub = None


def main():
	init()
	global sub
	sub = rospy.Subscriber('/ardrone/front/image_raw', Image, mainCallback)
	rospy.spin()
	cv2.destroyAllWindows()
	s.close()


def init():
	global data
	global mowayData
	global s
	s = None
	#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	#s.connect((TCP_IP, TCP_PORT))
	mowayData = cm.MowayState(0,0,0,0,0)
	rospy.init_node('logos')
	rospy.Rate(RATE)
	data = ms.ROSData()


def mainCallback(image):
	"""Glavni callback koji na temelju stanja poziva funkciju."""

	if stanje == Stanje.pocetno:
		startPocetno(image)
	elif stanje == Stanje.iteracija:
		startIteracija(image)


def startPocetno(image):
	"""Slikanje labirinta."""
	
	img = cv2.imread('vatro_sara.jpg')
	if img is not None:
		global sub
		sub.unregister()
		path = _findPath(img)
		print path
		# sub = rospy.Subscriber('/ardrone/front/image_raw', Image, mainCallback)
		cv2.waitKey(0)
	
	global data
	global mowayData
	global s

	npimg = ig.RospyImage(image).createNumpyImage()
	npimgCopy = ig.RospyImage(image).createNumpyImage()
	imgTagger = ms.ImageTagger(npimgCopy, True)

	# ovo ide kroz filter tako da su ostali podaci sigurni
	locator = ms.locateYourself(npimg, data, imgTagger)
	if locator is None:

		###################
		x, y = ig.CAMERA_SIZE
		cv2.circle(npimgCopy.getNumpyArray(), (x/2, y/2), 2, (0,0,0), 2)
		cv2.imshow('stream', npimgCopy.getNumpyArray())
		cv2.waitKey(1)
		return
	
	data.globalCoords.position.x = locator.x
	data.globalCoords.position.y = locator.y
	data.pubPoseGlobal.publish(data.globalCoords)
	
	x, y = ig.CAMERA_SIZE
	cv2.circle(npimgCopy.getNumpyArray(), (x/2, y/2), 2, (0,0,0), 2)
	cv2.imshow('stream', npimgCopy.getNumpyArray())
	cv2.waitKey(1)


def _findPath(img):
	"""Pronadje put iz spojene slike."""

	image = pf.Image(img)			# originalna slika
	w, h = image.getDimensions()

	startCoef = image.findStartCoef()
	endCoef = image.findEndCoef()
	if startCoef is None or endCoef is None:
		exit("Ulaz ili izlaz nisu pronadjeni!")

	image.optimalResize()
	(w, h) = image.getDimensions()	# dimenzije resizane slike

	start = _calculateValue(startCoef, (w, h))
	end = _calculateValue(endCoef, (w, h))
	
	mapHandler = pf.MapHandler(image.convertToGray())
	mapHandler.wallShades()		# prilagodjavanje mape
	finder = pf.PathFinder(mapHandler)

	path = finder.findPath(start, end)

	tPath = ms.tuplesPath(path)
	globalPath = ms.translatePath(tPath, start, end)

	globalPath = saveCornerDots(globalPath)

	pf.pointsToImage(img, path, image.getCoef())
	cv2.imshow('rezultat', img)
	return globalPath
	
	
def _calculateValue(pointCoef, dimensions):
	"""Racuna prave koordinate iz koeficijenata koordinata i dimenzija."""
	
	w, h = dimensions
	xc, yc = pointCoef
	return (int(xc * w), int(yc * h))


def startIteracija(image):
	"""Glavni callback koji vozi muveja kroz labirint."""

	# u data se moze dodat jos npimg i npimgCopy
	global data
	global mowayData
	global s

	npimg = ig.RospyImage(image).createNumpyImage()
	npimgCopy = ig.RospyImage(image).createNumpyImage()
	imgTagger = ms.ImageTagger(npimgCopy, True)

	# ovo ide kroz filter tako da su ostali podaci sigurni
	locator = ms.locateYourself(npimg, data, imgTagger)
	if locator is None:

		###################
		x, y = ig.CAMERA_SIZE
		cv2.circle(npimgCopy.getNumpyArray(), (x/2, y/2), 2, (0,0,0), 2)
		cv2.imshow('stream', npimgCopy.getNumpyArray())
		cv2.waitKey(1)
		return
	
	data.globalCoords.position.x = locator.x
	data.globalCoords.position.y = locator.y
	data.pubPoseGlobal.publish(data.globalCoords)
	
	#print "Quad: %+4d %+4d" % (locator.x, locator.y)

	#tezejPos, tezejAngle = ms.locateTezej(npimg, data, locator, imgTagger)

	#################
	x, y = ig.CAMERA_SIZE
	cv2.circle(npimgCopy.getNumpyArray(), (x/2, y/2), 2, (0,0,0), 2)
	cv2.imshow('stream', npimgCopy.getNumpyArray())
	cv2.waitKey(1)
	
	#if tezejPos is None or tezejAngle is None:
	#	return

	#print tezejPos, tezejAngle
	#print "##############"
	#cm.callMoway(tezejLoc[0], tezejLoc[1], mowayData, 1, [(30, -110)], s)


	#minotaurs = ms.locateMinotaurs(npimg, locator)

	# if na izlazu:
	# 	sub.unregister()
	#	kraj()


if __name__ == '__main__':
	main()
