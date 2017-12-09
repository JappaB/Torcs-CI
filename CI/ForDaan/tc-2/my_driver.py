from pytocl.driver import Driver
from pytocl.car import State, Command
import math
import pandas as pd
import torch
from torch.autograd import Variable
import os
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from sensorscreen import SensorScreen
from nnet_structure import DriverNet
from pytocl.controller import *
import random as r
import time
from collections import defaultdict
import operator
from timeit import default_timer as timer



def rotateVector (vector, degrees_rotation):
	theta = np.radians(degrees_rotation)
	rot_matrix = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
	rotated_vector = vector.dot(rot_matrix)
	return rotated_vector

def getRangesInsideBox (angles, box):
	return [limitRangeToBox(np.radians(a), *box) for a in angles]

def limitRangeToBox(angle, x, y):
	# bound to just top right corner since it is symmetric
	angle = abs(angle)
	thing = angle - math.pi/2
	if thing > 0.0: # if it aims down, reflect it in x-axis
		angle = math.pi/2 - thing
	e = 0.000001
	angle = np.clip(angle, e, math.pi/2 - e)
	rangeLimitedByX = x / math.cos(math.pi/2 - angle)
	rangeLimitedByY = y / math.cos(angle)
	limitedRange = min(rangeLimitedByX, rangeLimitedByY)/2.0
	return limitedRange

def calcWheelSlip (carSpeedMps, wheelSpinVel):
	frontSlip = 0.0
	rearSlip = 0.0
	if carSpeedMps > 0.001:
		slips = []
		for i in range(4):
			wheelSurfaceSpeed = wheelSpinVel[i] * 0.3306
			difference = wheelSurfaceSpeed - carSpeedMps
			proportionalDifference = difference / carSpeedMps
			slips.append(proportionalDifference)
		frontSlip = np.mean(slips[0:2]) # only front wheels for ABS
		rearSlip = np.mean(slips[2:4]) # only rear wheels for TCL
		avgSlip = np.mean(slips)
		#print("wheel speeds relative to ground: {} (avg: {:.2f})".format(' '.join(["{0:.2f}".format(i) for i in slips]), avgSlip))
	return frontSlip, rearSlip


def state_var(carstate):
	# Extract input_data
	curr_state = np.asarray([
		carstate.angle,
		0.0,#carstate.speed_x / (200.0/3.6), 
		carstate.speed_y, 
		carstate.speed_z ,
		carstate.distance_from_center, 
		carstate.z]
		+[bla / (200.0) for bla in list(carstate.distances_from_edge)
		#+[bla / (200.0) for bla in list(carstate.opponents)]
		])
	# Turn  input state into Torch variable
	inp_data = Variable(torch.from_numpy(curr_state).float())
	return inp_data

def state_var (carstate, maxRangeAngle):
	frame = np.asarray(
		[maxRangeAngle, carstate.speed_y]
		+[scan/200.0 for scan in carstate.distances_from_edge]
		)
	return Variable(torch.from_numpy(frame).float())

def state_var_excl(carstate):
	# Extract input_data
	curr_state = np.asarray([
		0.0,#carstate.speed_x *3.6,
		0.0,#carstate.distance_from_center, 
		0.0]#carstate.angle]
		+[bla for bla in list(carstate.distances_from_edge)
		#+[bla / (200.0) for bla in list(carstate.opponents)]
		])
	# Turn  input state into Torch variable
	inp_data = Variable(torch.from_numpy(curr_state).float())
	return inp_data

def state_var_incl(carstate):
	# Extract input_data
	curr_state = np.asarray([
		carstate.speed_x *3.6,
		carstate.distance_from_center, 
		carstate.angle]
		+[bla for bla in list(carstate.distances_from_edge)
		#+[bla / (200.0) for bla in list(carstate.opponents)]
		])
	# Turn  input state into Torch variable
	inp_data = Variable(torch.from_numpy(curr_state).float())
	return inp_data


def phero_var(samples):
	return Variable(torch.from_numpy(np.asarray(samples)).float())



class MyDriver(Driver):

	def __init__(self, path, contestant, logdata = False):
		super().__init__(logdata)
		self.yeaIFuckedUp = False
		self.lastMoved = 0.0
		self.backwardDistance = 0.0
		self.previousCarstate = 0
		self.previousCommand = None

		self.performance = 0.0
		self.startPos = 0
		self.avgBrake = 0.0

		# default angles
		self.rangeAngles = [-90, -75, -60, -45, -30, -20, -15, -10, -5, 0, 5, 10, 15, 20, 30, 45, 60, 75, 90]
		self.carSize = [1.94, 4.52]

		# Load model
		nnets = DriverNet.loadNetSet(path, contestant)
		self.steerNet = nnets['steer']
		self.speedNet = nnets['speed']

		# open sensor data viewer
		self.shouldDrawDebug = False
		if self.shouldDrawDebug:
			self.sensorScreen = SensorScreen()

		# get some stats
		self.shouldReportSimuTime = False
		self.shouldReportTime = False
		if self.shouldReportTime:
			self.startTime = time.time()
			self.timeTaken = defaultdict(lambda: 0.0)
			self.timer = timer()
			self.timerIndex = 0
			self.shit = 0

		# fucking up is great for natural selection
		self.canFuckUp = True
		self.canFuckUpBackwards = True
		self.canFuckUpSideways = True
		self.canFuckUpSprinting = False
		self.canFuckUpOffTrack = True
		self.canFuckUpMyCar = False
		self.canFuckUpSlowly = True
		self.canFuckUpStuck = True
		self.canFuckUpSteering = False
		self.avgSteeringChanges = 0.0
		self.steeringChanges = 0
		self.totalSteering = 0.0

		self.enableSafetyRepulsion = False
		self.shouldJustCruise = True
		# write data for training
		self.writeCsv = False
		if self.writeCsv:
			self.canFuckUp = False
			self.startCsv()

		# dampen steering to force smooth driving
		self.steering = 0.0
		self.steeringController = ProportionalController(0.1)

		# self.targetSpeed = 50.0
		# self.speedController = ProportionalController(0.1)

		# dampen pedal action to prevent jerky control of car
		self.pedalAction = 0.0
		self.pedalController = ProportionalController(0.01)

		# brake without locking up
		self.absTarget = -0.2 # real world value. works well
		self.absBrakePressure = 0.5
		self.absController = ProportionalController(0.25)

		# accelerate without slipping
		self.tclTarget = 0.1
		self.tclAccelPressure = 1.0
		self.tclController = ProportionalController(0.1)

		#ant colony feromone knowledge drop
		self.track_knowledge = defaultdict(lambda: 100.0)
		self.predictedCarRotation = 0.0
		self.carRotation = 0.0
		self.compass_prev = 0.0

	def on_shutdown(self):
		if self.writeCsv:
			self.csvFile.close()
		# Report performance to evoManager
		#print("PERFORMANCE:{}".format(self.performance))

		print("total steering: {:.2f} (per meter: {:.2f})".format(self.totalSteering, self.totalSteering/self.previousCarstate.distance_raced))
		return self.performance
		...

	# def on_skip (self, performance):
	# 	print("My performance: {}".format(performance))
	# 	print("My own performance: {}".format(self.performance))
	# 	...

	# @property
	# def range_finder_angles(self):
	# 	return self.rangeAngles
	# 	self.rangeAngles = []
	# 	roadWidth = 11
	# 	sensorFocus = 25
	# 	sampleSpread = (np.logspace(0, 1.0, base=sensorFocus, num=9) - 1)/(sensorFocus-1)
	# 	for i in range(19):
	# 		if i < 9:
	# 			angle = np.arctan((sampleSpread[i]*200.0) / (roadWidth/2))
	# 			angle = ((-math.pi/2.0) + angle) * 180/math.pi
	# 		elif i == 9:
	# 			angle = 0
	# 		elif i > 9:
	# 			angle = np.arctan((sampleSpread[(9-(i-9))]*200.0) / (roadWidth/2))
	# 			angle = ((math.pi/2.0) - angle) * 180/math.pi
	# 		self.rangeAngles.append(angle)
	# 	return self.rangeAngles



	def autoTransmission (self, carstate, targetSpeed, rearSlip):
		speedDifference = targetSpeed - carstate.speed_x
		isAccel = speedDifference > 1
		isDecel = speedDifference < -5
		rpm = carstate.rpm
		gear = carstate.gear
		upshiftRpm = 9400
		downshiftRpm = 7100
		maximumSlip = 0.1
		if gear < 6 and rearSlip < maximumSlip and (isAccel and rpm > upshiftRpm) or rpm > 9700:
			# print ("gear: {}, rpm: {}, upshift!".format(gear, rpm))
			return gear + 1
		elif gear > 1 and ((isDecel and rpm < downshiftRpm) or rpm < 5500):
			# print ("gear: {}, rpm: {}, downshift!".format(gear, rpm))
			return gear - 1
		# print ("gear: {}, rpm: {}".format(gear, rpm))
		if gear < 1: # in neutral
			return gear + 1
		return gear


	def steerToCenter (self, trackPos):
		steering = trackPos * -0.50
		steering = np.clip(steering, -1, 1)
		return steering


	def steerToCorner (self, angle):
		steering = angle / math.pi
		return steering


	def steerToFurthestRange (self, trackRanges):
		maxIndex = np.argmax(trackRanges)
		angle = -self.rangeAngles[maxIndex] * math.pi/180.0
		steering = angle / math.pi
		return steering


	def getPerformance (self, carstate):
		if carstate.distance_raced > 100:
			startPosCompensation = self.startPos * 10
			totalDistanceFromStartingtLine = carstate.distance_raced - startPosCompensation

			avgSpeed = totalDistanceFromStartingtLine / max(carstate.last_lap_time, carstate.current_lap_time)
			speedBonus = (avgSpeed/15) * min(totalDistanceFromStartingtLine, 500)
			crashSpeedPenalty = carstate.speed_x * 3.6 # in kph for extra negative incentive
			brakingBonus = crashSpeedPenalty * (1.0 - self.avgBrake) * 0.5 # give some slack when he was at least braking
			steeringPenalty = self.totalSteering/carstate.distance_raced * 1000
			performance = totalDistanceFromStartingtLine
			performance += speedBonus
			performance += brakingBonus
			performance -= crashSpeedPenalty
			performance -= steeringPenalty
			#performance = max(performance, 1.0) # prevent being randomized
		else:
			performance = carstate.distance_raced # no extra incentives for first 100m

		return performance


	def hasFuckedUp (self, carstate, previousCarstate, command, previousCommand):
		if not self.canFuckUp:
			return False

		if self.canFuckUpStuck:
			# When I haven't really moved in a while
			if carstate.speed_x > 3:
				self.lastMoved = carstate.current_lap_time
			if carstate.current_lap_time - self.lastMoved > 5.0:
				print("fucked up doing nothing after {:.1f}m".format(carstate.distance_raced))
				return True

		if self.canFuckUpSprinting:
			# when I reached 500m
			if carstate.distance_raced > 150 and carstate.distance_from_start > 300:
				startPosCompensation = self.startPos * 10
				totalDistanceFromStartingtLine = carstate.distance_raced - startPosCompensation
				avgSpeed = totalDistanceFromStartingtLine / max(carstate.last_lap_time, carstate.current_lap_time)
				print("reached end of sprint with avg speed: {:.1f}kmh".format(avgSpeed*3.6))
				return True
		
		if self.canFuckUpOffTrack:
			# When I run off the track
			if abs(carstate.distance_from_center) > 0.8:
				print("fucked up off the track after {:.1f}m".format(carstate.distance_raced))
				return True

		if self.canFuckUpSteering:
			# when I steer too eratically
			if np.sign(command.steering) != np.sign(previousCommand.steering):
				self.steeringChanges += 1
			self.avgSteeringChanges = (199.0 * self.avgSteeringChanges + self.steeringChanges) / 200.0
			print(self.avgSteeringChanges)
			if self.avgSteeringChanges > 0.5:
				return True

		if self.canFuckUpBackwards:
			# When I go backwards 10m
			distanceTraveled = carstate.distance_from_start - previousCarstate.distance_from_start
			if distanceTraveled < 0.0:
				self.backwardDistance -= max(distanceTraveled, -1) # positive
			elif distanceTraveled > 0.0:
				self.backwardDistance = 0.0 # reset
			if self.backwardDistance > 10.0:
				print("fucked up backwards after {:.1f}m".format(carstate.distance_raced))
				return True

		if self.canFuckUpSideways:
			# when I get into a giant slide
			if carstate.speed_y > 4:
				print("fucked up sliding around after {:.1f}m".format(carstate.distance_raced))
				return True

		if self.canFuckUpSlowly:
			# when I'm too slow
			if carstate.current_lap_time > 300:
				print("fucked up slowly after {:.1f}m".format(carstate.distance_raced))
				return True

		if self.canFuckUpMyCar:
			# when I bumped too hard
			if carstate.damage > 500:
				print("fucked up my shitty car after {:.1f}m".format(carstate.distance_raced))
				return True

		return False

	def repulsion_magnet(self, sensor, weight_of_sensor):
		score = np.subtract(1.0,sensor)
		score_to_power = np.power(score,3)		
		# print(score_to_power)
		# print(weight_of_sensor)
		mult_weight = np.multiply(score_to_power,weight_of_sensor)
		sum_over = np.sum(mult_weight)
		return sum_over
	def heat_avoiding_missile(self, carstate):
		'''more like a magnet'''
		#opponent sensor angles
		opponentAngles = [-math.pi/2 - (i+1) * math.pi/18 for i in range(36)]
		# element-wise subtraction to get range from car
		opponents = list(map(operator.sub, carstate.opponents, self.getRangesInsideCar(opponentAngles)))
		#from 90 degrees at the side of the car to the front
		weight_right_sensors = np.linspace(0.2,0.03,9)#TODO: make global
		weight_left_sensors = np.linspace(0.03,0.2,9)
		left_side_sensors = np.divide(opponents[9:18],200.0)
		right_side_sensors = np.divide(opponents[18:27],200.0)
		left_steer = self.repulsion_magnet(right_side_sensors,weight_right_sensors)
		right_steer = self.repulsion_magnet(left_side_sensors,weight_left_sensors)
		# print('left',left_steer)
		# print('right',right_steer)
		return left_steer+right_steer




	def getTrackCompass (self, carstate, prevAngle, steering):
		# compass angle in radians
		compass = (carstate.angle - prevAngle)
		if carstate.speed_y < 0.1 or True: ####### todo: kutten
			compass += self.predictedCarRotation

		# Predict rotation of car to compensate compass measurement 
		# next time step. We use a bicycle model to predict it
		# Calculates [-1,1] command.steering radians to degrees that the wheels turn 
		wheel_angle = (steering*21.0)*(math.pi/180)
		self.predictedCarRotation = 0.0
		if abs(wheel_angle) > 0.00000001:
			dist_since_last_update = carstate.speed_x *0.02
			# calculates the carturn from the degrees the wheel are turned
			wheelbase = 1.22 + 1.42
			turning_radius_in_m = wheelbase / math.sin(wheel_angle)
			# omtrek = math.pi*2*turning_radius_in_m
			# carturn = (dist_since_last_update/omtrk)
			self.predictedCarRotation = dist_since_last_update / turning_radius_in_m

		return compass

	def stain_track (self, placeOnTrack, pheromone):
		snif = self.track_knowledge[placeOnTrack]
		# if there is already are pheromone, average the two
		if snif < 50.0:
			pheromone = (snif + pheromone)/2.0
		self.track_knowledge[placeOnTrack] = pheromone

	def drop_pheromone(self, placeOnTrack, pheromone):
		self.stain_track(placeOnTrack-1, pheromone)
		self.stain_track(placeOnTrack, pheromone)
		self.stain_track(placeOnTrack+1, pheromone)

	def read_pheromones(self, carstate):
		reference_point = int(carstate.distance_from_start)
		# sample pheromonesspeedDifference, but only if pheromones have been dropped
		pheromones_sample = [self.track_knowledge[reference_point+i] for i in range(25) if self.track_knowledge[reference_point+i] < 50.0]
		pheromones_sample_mean = np.array(pheromones_sample).mean()
		# print('pheromones', pheromones_sample)
		# print(' pheromones mean', pheromones_sample_mean)
		return pheromones_sample_mean


	def getSteeringFromNet(self, nnetInput):
		steerNetOut = self.steerNet(nnetInput).data[0,0,0]
		return np.clip(steerNetOut, -1.0, 1.0)


	def getTargetSpeedFromNet(self, nnetInput):
		speedNetOut = self.speedNet(nnetInput).data[0,0,0]
		speedNetOut *= 200.0 # denormalize
		return speedNetOut


	def carSpeedControl (self, carstate, targetSpeed, frontSlip, rearSlip, steering):
		accel = 0.0
		brake = 0.0
		# speedDifference = targetSpeed - self.targetSpeed
		# speedDifference = np.clip(speedDifference, -100, 10)
		# self.targetSpeed += self.speedController.controlLazy(speedDifference)
		#print(targetSpeed*3.6)
		speedDifference = targetSpeed - carstate.speed_x
		speedDifference = np.clip(speedDifference, -5, 1.5)
		isDoingItWrong = np.sign(speedDifference) != np.sign(self.pedalAction) 
		if isDoingItWrong:
			self.pedalAction = 0.0
		self.pedalAction += self.pedalController.controlLazy(speedDifference)
		self.pedalAction = np.clip(self.pedalAction, -1, 1)
		if self.pedalAction > 0.0:
			accel = self.pedalAction
			if (accel > 0.6 or rearSlip > 0.05) and carstate.speed_x > 10: # TCL is crap at low speeds
				# activate traction control
				self.tclAccelPressure -= self.tclController.controlLazy(rearSlip - self.tclTarget)
				self.tclAccelPressure = np.clip(self.tclAccelPressure, 0.0, 1.0)
				self.pedalAction = self.tclAccelPressure
				accel = self.tclAccelPressure
		else:
			brake = -self.pedalAction
			if brake >= self.absBrakePressure or frontSlip < -0.05:
				# activate ABS
				self.absBrakePressure += self.absController.controlLazy(frontSlip - self.absTarget)
				self.absBrakePressure = np.clip(self.absBrakePressure, 0.0, 1.0)
				self.pedalAction = self.absBrakePressure
				brake = self.absBrakePressure

		accel = np.clip(accel, 0, max(1-abs(steering*3), 0.5))
		return accel, brake


	def controlSteering (self, steer, trackRepulsion):
		isDoingItWrong = np.sign(steer) != np.sign(self.steering) 
		if isDoingItWrong:
			self.steering = 0.0
		self.steering += self.steeringController.controlLazy(steer - self.steering)
		self.steering = np.clip(self.steering, -1.0, 1.0) - trackRepulsion
		return self.steering

	def measureTime (self):
		if not self.shouldReportTime:
			return
		diff = timer() - self.timer
		self.timeTaken[self.timerIndex] = (49.0 * self.timeTaken[self.timerIndex] + diff) / 50.0
		self.timerIndex += 1
		self.timer = timer()


	def repulseEdges (self, trackRanges, trackAngles):
		# element-wise subtraction to get range from car
		distancesFromCar = [max(a - b, 0.001) for a,b in zip(trackRanges, getRangesInsideBox(trackAngles, self.carSize))]
		weightBox = [0.5, 30]
		weights = getRangesInsideBox(trackAngles, weightBox)
		weightedDistances = [r/w for w,r in zip(weights, distancesFromCar)]
		constant = 1.0
		forces = [constant/(r**2) for r in weightedDistances]
		rotatedForceVectors = []
		for F,angle in zip(forces, trackAngles):
			vec = np.array([0.0, -F])
			rotatedForceVectors.append(rotateVector(vec, angle))
		forceVector = np.sum(np.array(rotatedForceVectors), axis=0)
		return forceVector


	def getCruiseSpeed(self, carstate, steering):
		#targetSpeed = 70 + (10 - carstate.race_position) * 5 # kph
		distances = carstate.distances_from_edge[5:14]
		# relatedAngles = self.rangeAngles[6:13]
		# weightBox = [0.05, 1.0]
		# weights = getRangesInsideBox(relatedAngles, weightBox)
		# weightedDistances = [w*r for w,r in zip(weights, distancesFromCar)]
		maxDist = np.max(distances) / (1 + (carstate.speed_x / 135)**2)
		customCurve = max(250*math.sqrt((max(maxDist, 51)-50)/50.0) - 150, 0)
		baseSpeed = maxDist + customCurve + 70
		steeringPenalty = 40 * math.sqrt(abs(steering))
		speed = baseSpeed - steeringPenalty
		speed = max(speed, 30)
		#print("{:.0f}m: {:.0f}kmh - {:.0f}kmh = {:.0f}kmh".format(maxDist, baseSpeed, steeringPenalty, speed))
		speed /= 3.6 # m/s
		return speed

	def getCruiseSteer(self, carstate):
		steer = (
			3.0 * self.steerToCorner(carstate.angle)
			+ 1.0*self.steerToCenter(carstate.distance_from_center)
			+ 4.0*self.steerToFurthestRange(carstate.distances_from_edge)
			#+ self.heat_avoiding_missile(carstate.opponents)
			)
		return np.clip(steer, -1.0, 1.0)

	def cruise (self, carstate):
		frontSlip, rearSlip = calcWheelSlip(carstate.speed_x, carstate.wheel_velocities)
		trackRepulsion = 0.0
		if abs(carstate.angle) < math.pi/2.0: # only when facing right way
			trackRepulsion = self.repulseEdges(carstate.distances_from_edge, self.rangeAngles)[0]
		#print(force)
		command = Command()
		command.steering = self.getCruiseSteer(carstate) - trackRepulsion
		targetSpeed = self.getCruiseSpeed(carstate, command.steering)
		command.accelerator, command.brake = self.carSpeedControl(carstate, targetSpeed, frontSlip, rearSlip, command.steering)
		command.gear = self.autoTransmission(carstate, targetSpeed, rearSlip)

		if self.writeCsv:
			if abs(carstate.distance_from_center) > 1:
				print("WE FUCKED UP YO")
				time.sleep(1)
			maxAngle = np.radians(self.rangeAngles[np.argmax(carstate.distances_from_edge)])
			self.writeCsvRow(carstate, command, maxAngle, targetSpeed)
		return command

	def hasFuckedUp (self, carstate, previousCarstate):

		# When I run off the track
		if abs(carstate.distance_from_center) > 0.8:
			print("fucked up off the track after {:.1f}m".format(carstate.distance_raced))
			return True

		# When I haven't really moved in a while
		if carstate.speed_x > 3:
			self.lastMoved = carstate.current_lap_time
		if carstate.current_lap_time - self.lastMoved > 5.0:
			print("fucked up doing nothing after {:.1f}m".format(carstate.distance_raced))
			return True

		# When I go backwards 10m
		distanceTraveled = carstate.distance_from_start - previousCarstate.distance_from_start
		if distanceTraveled < 0.0:
			self.backwardDistance -= max(distanceTraveled, -1) # positive
		elif distanceTraveled > 0.0:
			self.backwardDistance = 0.0 # reset
		if self.backwardDistance > 10.0:
			print("fucked up backwards after {:.1f}m".format(carstate.distance_raced))
			return True

		# when I'ḿ too slow
		if carstate.current_lap_time > 300:
			print("fucked up slowly after {:.1f}m".format(carstate.distance_raced))
			return True

		return False

	def heat_avoiding_missile(self, opponents_finder):
		front_lasers = [opponents_finder[16:20]]
		opp_dist = np.min(front_lasers)
		maxIndex = np.argmax(front_lasers)

		# print('16',opponents_finder[16])
		
		# print('17',opponents_finder[17])

		# print('18',opponents_finder[18])

		# print('19',opponents_finder[19])

		steering = 0
		# min_opp_dist = 100
		# if opp_dist < min_opp_dist:
		# 	if:
				
		# 		action = clip(min_opp_dist - opp_dist, 0, min_opp_dist)
		# 		action /= min_opp_dist
		# 		action *= action
		# 		angle = -self.rangeAngles[maxIndex] * math.pi/180.0)
		# 		steering = angle / math.pi
		# 		print('steer to: ', maxIndex)

		return steering


	# def drop_knowledge(self, carstate, command):
	# 	# {dist from start (int): angle of steering + angle(bycicle model),....}

	# 	self.total_angle = carstate.angle+((math.pi*command.steering)/180.0)
	# 	# Interpolation TODO?
	# 	print('dist raced', carstate.distance_from_start)
	# 	#drop knowledge every 5m (TODO: Only if you are front car which drives savely)
	# 	if (int(carstate.distance_from_start)) and (carstate.last_lap_time ==0):
	# 		self.total_angle_prev = self.total_angle
	# 		self.track_knowledge[int(carstate.distance_from_start)] = self.total_angle
	# 		# print('lap1',command.steering)
	# 	# second round first car starts using the knowledge as well
	# 	elif(carstate.last_lap_time != 0):
	# 		print('lookup steering angle last round:')
	# 		print('lookup',(self.track_knowledge[str(int(carstate.distance_from_start))]
	# 			*180/math.pi)

	# 		command.steering = (
	# 			self.steerToCenter(carstate.distance_from_center) +
	# 			(self.track_knowledge[str(int(carstate.distance_from_start))]*180/(math.pi)))
	# 		print('lap2',command.steering)


		# print('prev_lap_time',carstate.last_lap_time) 
		# print(self.track_knowledge)


	def drive(self, carstate)-> Command:
		if self.shouldJustCruise:
			return self.cruise(carstate)

		command = Command()

		if self.previousCarstate == 0:
			self.previousCarstate = carstate
			self.previousCommand = command
			self.startLapTime = carstate.current_lap_time
			self.startPos = carstate.race_position
			self.trackRotation = 0.0
			self.prevAngle = carstate.angle
			self.totalCompass = 0.0
			self.hasSeenStart = False

		# In evolution mode:
		if self.yeaIFuckedUp:
			command.steering = -1.0
			return command


		self.timerIndex = 0
		self.measureTime()

		frontSlip, rearSlip = calcWheelSlip(carstate.speed_x, carstate.wheel_velocities)

		## steering ##
		# get repulsed from track edges as a safety net
		trackRepulsion = 0.0
		if self.enableSafetyRepulsion and abs(carstate.angle) < math.pi/2.0: # only when facing right way
			force = self.repulseEdges(carstate.distances_from_edge, self.rangeAngles)
			trackRepulsion = force[0]

		maxAngle = np.radians(self.rangeAngles[np.argmax(carstate.distances_from_edge)])
		nnetInput = state_var(carstate, maxAngle)
		steerTarget = self.getSteeringFromNet(nnetInput)
		command.steering = self.controlSteering(steerTarget, trackRepulsion)
		self.measureTime()

		## speed ##
		targetSpeed = self.getCruiseSpeed(carstate, command.steering)
		#targetSpeed = self.getTargetSpeedFromNet(nnetInput)
		#targetSpeed = max(targetSpeed, targetSpeedBaseLine)
		self.measureTime()

		# guarantee smooth driving with complex controller
		command.accelerator, command.brake = self.carSpeedControl(carstate, targetSpeed, frontSlip, rearSlip, command.steering)
		command.gear = self.autoTransmission(carstate, targetSpeed, rearSlip)

		# compass = self.getTrackCompass(carstate, self.previousCarstate.angle, command.steering)
		# track_angle = compass - self.compass_prev
		# self.compass_prev = compass
		# if not self.hasSeenStart:
		# 	self.hasSeenStart = int(carstate.distance_from_start)==0
		# if carstate.last_lap_time == 0.0 and self.hasSeenStart:
		# 	self.trackRotation += carstate.angle - self.prevAngle
		# 	self.prevAngle = carstate.angle
		# 	self.carRotation += self.predictedCarRotation
		# 	self.totalCompass += compass
		#print("\t\t{:.4f}\t\t{:.4f}\t\t{:.4f}".format(self.carRotation, self.trackRotation, self.totalCompass))
		#print("{:.6f}".format(carstate.speed_y))
		# placeOnTrack = int(carstate.distance_from_start)
		# self.drop_pheromone(placeOnTrack, track_angle)

		# if placeOnTrack % 50 == 0 and carstate.last_lap_time < 0.1:
		# 	for i in range(placeOnTrack-25, placeOnTrack+25):
		# 		relIndex = i - placeOnTrack
		# 		print("{}:\t{:.4f}".format(relIndex, self.track_knowledge[i]))

		# if carstate.last_lap_time > 0.1:
		# 	totalAngle = 0.0
		# 	for i in range(len(self.track_knowledge)-28):
		# 		totalAngle += self.track_knowledge[i]
		# 	print("total angle: {:.3f}".format(totalAngle))


		# Retire when you fucked up. When half of the 
		# participants have fucked up, the race ends.
		self.avgBrake = (self.avgBrake * 49 + command.brake) / 50
		self.performance = self.getPerformance(carstate)
		if self.hasFuckedUp(carstate, self.previousCarstate, command, self.previousCommand):
			command.meta = 2
			self.yeaIFuckedUp = True

			# startPosCompensation = self.startPos * 10
			# totalDistanceFromStartingtLine = carstate.distance_raced - startPosCompensation

			# avgSpeed = totalDistanceFromStartingtLine / max(carstate.last_lap_time, carstate.current_lap_time)
			# speedBonus = (avgSpeed/50) * min(totalDistanceFromStartingtLine, 500)
			# print("dist: {:.0f} speed: {:.1f} bonus: {:.1f}".format(totalDistanceFromStartingtLine, avgSpeed*3.6, speedBonus))
			print("total steering: {:.2f} (per meter: {:.2f})".format(self.totalSteering, self.totalSteering/carstate.distance_raced))
			if self.shouldReportSimuTime:
				# report simulation speed (it's about 5x)
				realTimePassed = time.time() - self.startTime
				simuTimePassed = carstate.current_lap_time - self.startLapTime
				simuFactor = simuTimePassed / realTimePassed
				print("simulation time factor: {:.2f}x".format(simuFactor))

		if self.writeCsv:
			maxAngle = np.radians(self.rangeAngles[np.argmax(carstate.distances_from_edge)])
			self.writeCsvRow(carstate, command, maxAngle, targetSpeed)

		if self.shouldReportTime:
			if self.shit%50 == 0:
				times = [time * 1000 for time in self.timeTaken.values()]
				times = ["{:.1f}".format(time) for time in times]
				times = "\t".join(times)
				print("timers (ms): ", times)
			self.shit += 1
		
		if self.shouldDrawDebug:
			self.sensorScreen.update(carstate, self.rangeAngles)

		self.totalSteering += math.sqrt(abs(command.steering))
		self.previousCarstate = carstate
		self.previousCommand = command
		return command


	def startCsv (self):
		headers = ([
			"targetSpeed",
			"accel",
			"brake",
			"steer",
			"angle",
			"maxRangeAngle",
			"curLapTime",
			"distFromStart",
			"distRaced",
			"gear",
			"lastLapTime",
			"racePos",
			"rpm",
			"speedX",
			"speedY",
			"speedZ",
			"trackPos",
			"z",
			"wheelSpinVel01",
			"wheelSpinVel02",
			"wheelSpinVel03",
			"wheelSpinVel04",
			"track00","track01","track02","track03","track04",
			"track05","track06","track07","track08","track09",
			"track10","track11","track12","track13","track14",
			"track15","track16","track17","track18",
			"oppos00","oppos01","oppos02","oppos03","oppos04",
			"oppos05","oppos06","oppos07","oppos08","oppos09",
			"oppos10","oppos11","oppos12","oppos13","oppos14",
			"oppos15","oppos16","oppos17","oppos18","oppos19",
			"oppos20","oppos21","oppos22","oppos23","oppos24",
			"oppos25","oppos26","oppos27","oppos28","oppos29",
			"oppos30","oppos31","oppos32","oppos33","oppos34",
			"oppos35"])
		self.csvPath = 'cruise.csv'
		if not os.path.exists(self.csvPath):
			self.csvFile = open(self.csvPath, 'w+')
			header = ';'.join(headers)
			self.csvFile.write(header + '\n')
		else:
			self.csvFile = open(self.csvPath, 'a')


	def writeCsvRow (self, carstate, command, maxRangeAngle, targetSpeed):
		rowData = []
		rowData.append(targetSpeed)
		rowData.append(command.accelerator)
		rowData.append(command.brake)
		rowData.append(command.steering)
		rowData.append(carstate.angle)
		rowData.append(maxRangeAngle)
		rowData.append(carstate.current_lap_time)
		rowData.append(carstate.distance_from_start)
		rowData.append(carstate.distance_raced)
		rowData.append(carstate.gear)
		rowData.append(carstate.last_lap_time)
		rowData.append(carstate.race_position)
		rowData.append(carstate.rpm)
		rowData.append(carstate.speed_x)
		rowData.append(carstate.speed_y)
		rowData.append(carstate.speed_z)
		rowData.append(carstate.distance_from_center)
		rowData.append(carstate.z)
		for thing in carstate.wheel_velocities:
			rowData.append(thing) 
		for thing in carstate.distances_from_edge:
			rowData.append(thing) 
		for thing in carstate.opponents:
			rowData.append(thing)
		row = ';'.join([str(thing) for thing in rowData])
		self.csvFile.write(row + '\n')