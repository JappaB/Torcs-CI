from collections import namedtuple
from tkinter import *
import math
import numpy as np

Point = namedtuple('Point', 'x y')
Box = namedtuple('Box', 'width height')

class SensorScreen:
	def __init__ (self):
		self.trackAngleCompensation = 0.0
		self.trackPosCompensation = 0.0
		self.screenSize = Box(600, 1000)
		self.carSize = Box(1.94, 4.52)
		self.carCenter = Point(
				0.5*self.screenSize.width, 
				0.9*self.screenSize.height)
		self.zoom = 4.0
		self.screenElements = {}
		self.window = Tk()
		self.canvas = Canvas(
				self.window, 
				width=self.screenSize.width, 
				height=self.screenSize.height)
		self.canvas.pack()
		self.createScreenElements()

	def createScreenElements(self):
		# create screen elements in drawing order
		# green background
		element = self.canvas.create_rectangle(
				0, 
				0, 
				self.screenSize.width+1, 
				self.screenSize.height+1, 
				fill='#018E0E')
		self.screenElements['background'] = element

		# grey road
		element = self.canvas.create_polygon(
				[0,0,0,0], 
				fill='#777', 
				smooth='1')
		self.screenElements['road'] = element

		# 2 white sidelines of track
		self.screenElements['sidelines'] = []
		for i in range(2):
			element = self.canvas.create_line(
					self.carCenter, 
					self.carCenter, 
					fill='#CCC', 
					width='3', 
					smooth='1')
			self.screenElements['sidelines'].append(element)

		# the 19 faint lines showing the range finders
		self.screenElements['rangeLines'] = []
		for i in range(19):
			element = self.canvas.create_line(
					self.carCenter, 
					self.carCenter, 
					fill='#888')
			self.screenElements['rangeLines'].append(element)

		# 36 red arcs indicating possible enemy position
		self.screenElements['enemyArcs'] = []
		for i in range(36):
			thetaStart = -math.pi/2 - (i+1) * math.pi/18
			thetaStart *= 180.0/math.pi
			extent = 10
			element = self.canvas.create_arc(
				self.carCenter, 
				self.carCenter, 
				start=thetaStart, 
				extent=extent, 
				style='arc', 
				width='2', 
				outline='red')
			self.screenElements['enemyArcs'].append(element)

		# the car
		self.screenElements['car'] = self.canvas.create_rectangle(
			self.carCenter.x - 0.5 * self.carSize.width * self.zoom, 
			self.carCenter.y - 0.5 * self.carSize.height * self.zoom, 
			self.carCenter.x + 0.5 * self.carSize.width * self.zoom, 
			self.carCenter.y + 0.5 * self.carSize.height * self.zoom, 
			fill="blue")
		#self.screenElements['car'] = self.canvas.create_polygon()


	def updateTrackDisplay (self, trackdata, rangeAngles):
		sidelinePoints = []
		sidelinePoints.append([]) # left
		sidelinePoints.append([]) # right
		roadPoints = []
		maxIndex = np.argmax(trackdata)

		for i in range(19):
			# update range lines
			theta = rangeAngles[i] * math.pi/180.0
			# keep track angle constant angle in screen
			theta += self.trackAngleCompensation
			R = [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]];
			distance = trackdata[i] * self.zoom
			secondPoint = np.array([0,-distance])
			secondPoint = (R @ secondPoint) + self.carCenter
			self.canvas.coords(
					self.screenElements['rangeLines'][i], 
					*self.carCenter, 
					*tuple(secondPoint))

			# save for road surface polygon
			roadPoints.append(secondPoint)

			# assign it to left or right side of track
			# left/right are split by the furthest scan
			if i <= maxIndex and trackdata[i] < 200:
				sidelinePoints[0].append(secondPoint)
			elif i > maxIndex and trackdata[i] < 200:
				sidelinePoints[1].append(secondPoint)

			# draw furthest scan in a different color
			isFurthestScan = i == maxIndex
			color = '#888'
			if isFurthestScan:
				color = 'blue'
			self.canvas.itemconfig(
					self.screenElements['rangeLines'][i], 
					fill=color)

		# update road polygon
		magic = [e for l in roadPoints for e in l]
		self.canvas.coords(
				self.screenElements['road'], 
				magic)

		# update white side lines
		for i in range(2):
			magic = [e for l in sidelinePoints[i] for e in l]
			if len(magic) < 4:
				magic = [*self.carCenter, *self.carCenter]
			self.canvas.coords(
					self.screenElements['sidelines'][i], 
					magic)

		# update background to force redraw
		self.canvas.coords(
				self.screenElements['background'], 
				[0, 0, self.screenSize.width+1, self.screenSize.height+1])

		self.canvas.coords(
				self.screenElements['car'],
				self.carCenter.x - 0.5 * self.carSize.width * self.zoom, 
				self.carCenter.y - 0.5 * self.carSize.height * self.zoom, 
				self.carCenter.x + 0.5 * self.carSize.width * self.zoom, 
				self.carCenter.y + 0.5 * self.carSize.height * self.zoom)

	def updateEnemyDisplay (self, opponents):
		for i in range(36):
			distance = opponents[i]
			if distance > 199.9: # no enemy in range
				distance = 0.0
			distance *= self.zoom
			self.canvas.coords(
					self.screenElements['enemyArcs'][i],
					self.carCenter.x-distance, 
					self.carCenter.y-distance,
					self.carCenter.x+distance, 
					self.carCenter.y+distance)

	def update (self, carState, rangeAngles):
		self.trackAngleCompensation = carState.angle

		# keep center of track at constant point on screen
		# pray track is 11m wide
		trackOffsetPixels = 5.5 * carState.distance_from_center * self.zoom
		self.carCenter = Point(
				0.5*self.screenSize.width - trackOffsetPixels, 
				0.9*self.screenSize.height)
		self.updateTrackDisplay(carState.distances_from_edge, rangeAngles)
		self.updateEnemyDisplay(carState.opponents)
		if self.window.state() == 'normal':
			self.canvas.update()
