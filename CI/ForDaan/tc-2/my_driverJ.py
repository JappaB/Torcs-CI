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

def brakeAbs (brakePressure, frontSlip):
	absTarget = -0.25
	absControl = 0.25
	brakePressure += (frontSlip - absTarget) * absControl
	brakePressure = np.clip(brakePressure, 0.0, 1.0)
	return brakePressure

def accelTcl (accelPressure, rearSlip):
	tclTarget = 0.3
	tclControl = 0.25
	accelPressure -= (rearSlip - tclTarget) * tclControl
	accelPressure = np.clip(accelPressure, 0, 1)
	return accelPressure

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

def autoTransmission (gear, rpm, rearSlip):
	upshiftRpm = 9600
	downshiftRpm = 3500
	maximumSlip = 0.1
	if gear < 6 and rpm > upshiftRpm and rearSlip < maximumSlip:
		#print ("gear: {}, rpm: {}, upshift!".format(gear, rpm))
		return gear + 1
	elif gear > 1 and rpm < downshiftRpm:
		#print ("gear: {}, rpm: {}, downshift!".format(gear, rpm))
		return gear - 1
	#print ("gear: {}, rpm: {}".format(gear, rpm))
	if gear < 1: # in neutral
		return gear + 1
	return gear


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

def phero_var(samples):
	return Variable(torch.from_numpy(np.asarray(samples)).float())



class MyDriver(Driver):

	def __init__(self, path, contestant, logdata = False):
		super().__init__(logdata)
		self.yeaIFuckedUp = False
		self.lastMoved = 0.0
		self.backwardDistance = 0.0
		self.previousCarstate = 0
		self.topSpeed = 0.0
		self.performance = 0.0
		self.startPos = 0
		# default angles, will be overwritten by range_finder_angles()
		self.rangeAngles = [-90, -65, -50, -35, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 35, 50, 65, 90]
		# Load model
		nnets = DriverNet.loadNetSet(path, contestant)
		self.steerNet = nnets['steer']
		self.speedNet = nnets['speed']
		self.pheroNet = nnets['phero']
		# # open sensor data viewer
		# self.sensorScreen = SensorScreen()
		# remembering gas and brake pedals
		#self.startCsv()
		self.steering = 0.0
		self.steeringController = ProportionalController(0.1)

		self.pedalController = ProportionalController(0.5)

		self.absTarget = -0.2 # real world value. works well
		self.absBrakePressure = 0.5
		self.absController = ProportionalController(0.25)

		self.tclTarget = 0.2
		self.tclAccelPressure = 1.0
		self.tclController = CompositeController(
			ProportionalController(0.1),
			DerivativeController(0.02 * 0.25 * 0.5)
		)

		# ant colony feromone knowledge drop
		self.track_knowledge = defaultdict(lambda: 100.0)
		self.compass_prev = 0.0

		# unstuck
		self.offtrack_timestamp = 0.0
		self.kiss_boarding_timestamp = 0.0
		self.unstucking = False
		self.going_backwards = False 
		self.going_bacwards_timestamp = 0.0
	def on_shutdown(self):
		#self.csvFile.close()
		# Report performance to evoManager
		#print("PERFORMANCE:{}".format(self.performance))
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


	def steerToCenter (self, trackPos):
		steering = trackPos * -0.50
		steering = np.clip(steering, -1, 1)
		return steering


	def steerToCorner (self, angle):
		steering = angle*5 / math.pi
		return steering


	def steerToFurthestRange (self, trackRanges):
		maxIndex = np.argmax(trackRanges)
		angle = -self.rangeAngles[maxIndex] * math.pi/180.0
		steering = angle / math.pi
		return steering


	def cruise (self, carstate):
		command = Command()
		steer = getCruiseSteer(carstate)
		command.gear = self.autoTransmission(carstate.gear, carstate.rpm, 0.0)
		targetSpeed = self.getCruiseSpeed(carstate, command.steering)



		return command

	def hasFuckedUp (self, carstate, previousCarstate):

		#TODO WEGHALEN!!!
		return False 
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

		# when I'm too slow
		if carstate.current_lap_time > 300:
			print("fucked up slowly after {:.1f}m".format(carstate.distance_raced))
			return True

		# # when I bumped too hard
		# if carstate.damage > 500:
		# 	print("fucked up my shitty car after {:.1f}m".format(carstate.distance_raced))
		# 	return True

		return False


	def getCruiseSpeed(self, carstate, steering):
		targetSpeed = 200 + (self.startPos) * 3.5 # kph
		# targetSpeed = 90#########
		# targetSpeed -= 100 * abs(steering)################
		targetSpeed -= (100 - np.max(carstate.distances_from_edge))/2
		targetSpeed = max(targetSpeed, 30)
		targetSpeed /= 3.6 # m/s
		return targetSpeed
	
	def rotation_matrix(vector, degrees_rotation):

		theta = np.radians(degrees_rotation)
		rot_matrix = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
		rotated_vector = vector.dot(rot_matrix)
		return rotated_vector

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

		opponents = list(carstate.opponents)

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





	# 	return steer, brake


	def drop_knowledge(self, carstate, command):
		# bicycle model
		# Calculates [-1,1] command.steering radians to degrees that the wheels turn 
		wheel_angle = (command.steering*21.0)*(math.pi/180) 
		dist_since_last_update = carstate.speed_x *0.02
		# calculates the carturn from the degrees the wheel are turned
		wheelbase = 1.22 +1.42 #TODO make global
		# carturn_per_m = (wheelbase/(math.sqrt(2-2*math.cos(2*wheel_angle+0.0000000001))))
		#TODO: Div by zero
		turning_radius_in_m = ((wheelbase)/math.sin(wheel_angle+0.000000000000000000000000000000000001))
		# omtrek = math.pi*2*turning_radius_in_m
		# carturn = (dist_since_last_update/omtrk)
		carturn = dist_since_last_update/turning_radius_in_m
		#compass angle in radians
		self.compass = carstate.angle-carturn
		# self.total_angle = carstate.angle
		# print('steer', command.steering)
		# print('radius',turning_radius_in_m)
		# print('angle', carstate.angle)
		# print('steeringwheel_to_wheel', wheel_angle)
		# print('carturn', carturn)
		# for first time step, when track_angle can't be computed
		self.track_angle = self.compass-self.compass_prev
		# print('compass', self.compass)
		# print('compass_prev', self.compass_prev)
		# print('\t\t\ttrack_angle', self.track_angle)
		# if there already are pheromones:
		if self.track_knowledge[int(carstate.distance_from_start)] < 50.0:
			previous_pheromone = self.track_knowledge[int(carstate.distance_from_start)]
			self.track_knowledge[int(carstate.distance_from_start)] = (previous_pheromone + self.track_angle)/2.0
		else:
			self.track_knowledge[int(carstate.distance_from_start)] = self.track_angle
		self.compass_prev = self.compass


	def read_pheromones(self, carstate):
		reference_point = int(carstate.distance_from_start)
		# sample pheromonesspeedDifference, but only if pheromones have been dropped
		pheromones_sample = [self.track_knowledge[reference_point+i] for i in range(25) if self.track_knowledge[reference_point+i] < 50.0]
		pheromones_sample_mean = np.array(pheromones_sample).mean()
		# print('pheromones', pheromones_sample)
		# print(' pheromones mean', pheromones_sample_mean)
		return pheromones_sample_mean


		# print('prev_lap_time',carstate.last_lap_time) 
		# print(self.track_knowledge)


	def getCruiseSteer(self, carstate):
		steer = (
			self.steerToCorner(carstate.angle) + 
			self.steerToCenter(carstate.distance_from_center) + 
			self.steerToFurthestRange(carstate.distances_from_edge)+
			self.heat_avoiding_missile(carstate))
		return np.clip(steer, -1.0, 1.0)


	def getSteeringFromNet(self, nnetInput):
		steerNetOut = self.steerNet(nnetInput).data[0,0,0]
		return np.clip(steerNetOut, -1.0, 1.0)


	def getTargetSpeedFromNets(self, carstate, pheromones):
		pheroInput = phero_var(pheromones)
		nnetInput = state_var(carstate)

		# drive with neural nets
		speedNetOut = self.speedNet(nnetInput).data[0,0,0]
		pheroNetOut = self.pheroNet(pheroInput).data[0,0,0]

		# weigh shared control of target speed from both networks
		sampleHits = list(filter(lambda x: x != 100.0, pheromones))
		ratioSampleHits = len(sampleHits) / 20.0
		pheromoneRelevance = 0.5 * ratioSampleHits
		pheromoneRelevance = 0.0 # temp
		targetSpeed = pheromoneRelevance * pheroNetOut + (1.0-pheromoneRelevance) * speedNetOut
		return targetSpeed


	def carSpeedControl (self, carstate, targetSpeed, frontSlip, rearSlip):
		accel = 0.0
		brake = 0.0
		speedDifference = targetSpeed - carstate.speed_x
		pedalAction = self.pedalController.controlLazy(speedDifference)
		pedalAction = np.clip(pedalAction, -1, 1)
		if pedalAction > 0.0:
			accel = pedalAction
			if (accel > 0.6 or rearSlip > 0.05) and carstate.speed_x > 3: # TCL is crap at low speed
				# activate traction control
				self.tclAccelPressure -= self.tclController.controlLazy(rearSlip - self.tclTarget)
				self.tclAccelPressure = np.clip(self.tclAccelPressure, 0.0, 1.0)
				accel = self.tclAccelPressure
		else:
			brake = -pedalAction
			if brake >= self.absBrakePressure or frontSlip < -0.05:
				# activate ABS
				self.absBrakePressure += self.absController.controlLazy(frontSlip - self.absTarget)
				self.absBrakePressure = np.clip(self.absBrakePressure, 0.0, 1.0)
				brake = self.absBrakePressure
		return accel, brake


	def controlSteering (self, steer):
		self.steering += self.steeringController.controlLazy(steer - self.steering)
		self.steering = np.clip(self.steering, -1.0, 1.0)
		return self.steering



	def unstuck(self, carstate, command):
		# Unstucking (WIP)
		# Only laser range finders don't work anymore
		lasorz = list(carstate.distances_from_edge)
		
		# if offtrack, drive back to center
		if all(laser == -1.0 for laser in lasorz) and carstate.speed_x>2.5:
			# self.offtrack_timestamp = carstate.current_lap_time
			print('angle', carstate.angle)
			print('unstuck lasorz:',lasorz)
			print('distance to center:')
			# drive relatively straight back to the track
			# pref_driving_angle = 0.35 #rad
			steer = (self.steerToCorner(carstate.angle) + 
					self.steerToCenter(carstate.distance_from_center))
			command.steering = np.clip(steer,-1.0,1.0)


		if carstate.speed_x > 2.5 or carstate.speed_x < -6:
			self.hasHadSpeedAt = carstate.current_lap_time
		if carstate.speed_x < 0.0:
			command.steering *= -1

		timeSinceBackwards = carstate.current_lap_time - self.goBackwardsAt
		stuckTime = carstate.current_lap_time - self.hasHadSpeedAt 
		if stuckTime > 1.5 and stuckTime < 4 and carstate.current_lap_time > 5 and timeSinceBackwards > 6:
			self.goBackwardsAt = carstate.current_lap_time

		if timeSinceBackwards < 2.5:
			self.gear = -1

		# # if against boarding (or other cars), drive back to center (laptime >1.0 so it doesnt go backwards from the start)
		# if (carstate.speed_x < 2.5 and carstate.current_lap_time >1.0) or self.going_backwards == True:
		# 	if self.unstucking == False:
		# 		self.kiss_boarding_timestamp = carstate.current_lap_time
		# 	self.unstucking = True
			
		# 	print('going too slowly')

		# 	if ((carstate.current_lap_time - self.kiss_boarding_timestamp) > 1.5 and
		# 	 (carstate.current_lap_time - self.going_bacwards_timestamp > 6.0)):
		# 		print('now I will fucking go backwards')
		# 		command.gear = -1
		# 		command.accelerator = 1.0
		# 		command.steering *= -1
		# 		if self.going_backwards == False:
		# 			self.going_bacwards_timestamp = carstate.current_lap_time
		# 		self.going_backwards = True
		# 		if (carstate.current_lap_time - self.kiss_boarding_timestamp) > 3.0:
		# 			self.unstucking = False
		# 			self.going_backwards = False
		# else:
		# 	self.unstucking = False

		return command


	def drive(self, carstate)-> Command:
		if self.previousCarstate == 0:
			self.startTime = time.time()
			self.startLapTime = carstate.current_lap_time
			self.previousCarstate = carstate
			self.startPos = carstate.race_position
			self.hasHadSpeedAt = carstate.current_lap_time
			self.goBackwardsAt = carstate.current_lap_time
		command = Command()

		# In evolution mode:
		# Race casually after you fucked up
		if self.yeaIFuckedUp:
			# get the fuck off the track
			if abs(carstate.distance_from_center) < 1.0:
				command.accelerator = 0.3
			else:
				command.brake = 1.0
			steer = self.steerToCenter(carstate.distance_from_center) * 5
			if carstate.speed_x > 0.0:
				steer *= -1 # reverse driving
			command.steering = steer
			return command
			#return self.cruise(carstate)

		#self.drop_knowledge(carstate, command)

		# sample pheromones
		frontSlip, rearSlip = calcWheelSlip(carstate.speed_x, carstate.wheel_velocities)
		steer = self.getCruiseSteer(carstate)
		command.steering = self.controlSteering(steer)
		targetSpeed = self.getCruiseSpeed(carstate, command.steering) # cruise for now to train steering net
		# if np.max(carstate.distances_from_edge) > 199.0:
		# 	targetSpeed = 100


		#steer = self.getSteeringFromNet(state_var(carstate))
		#targetSpeed = self.getTargetSpeedFromNets(carstate, pheromones)



		command.accelerator, command.brake = self.carSpeedControl(carstate, targetSpeed, frontSlip, rearSlip)
		command.gear = autoTransmission(carstate.gear, carstate.rpm, rearSlip)


		# Retire when you fucked up. When half of the 
		# participants have fucked up, the race ends.
		self.performance = carstate.distance_raced
		if self.hasFuckedUp(carstate, self.previousCarstate) or False: ##################
			command.meta = 2
			self.yeaIFuckedUp = True
			if carstate.distance_raced > 100 and carstate.distance_raced < 3000:
				startPosCompensation = self.startPos * 10
				avgSpeed = carstate.distance_raced / carstate.current_lap_time # add early incentive for speed
				speedBonus = min(avgSpeed, 60.0) # passing startfinish can fuck it up
				crashSpeedPenalty = carstate.speed_x * 3.6 # in kph for extra negative incentive
				brakingBonus = crashSpeedPenalty * (1.0 - command.brake) * 0.5 # give some slack when he was at least braking
				self.performance -= startPosCompensation
				#self.performance += speedBonus
				#self.performance -= crashSpeedPenalty
				#self.performance += brakingBonus
				self.performance = max(self.performance, 1.0) # prevent being randomized
				# # report simulation speed (it's about 5x)
				# realTimePassed = time.time() - self.startTime
				# simuTimePassed = carstate.current_lap_time - self.startLapTime
				# simuFactor = simuTimePassed / realTimePassed
				# print("simulation time factor: {:.2f}x".format(simuFactor))


		#self.writeCsv(carstate, command)
		# if carstate.current_lap_time <= 1.0:
		# 	print('total angle track:', np.sum(list(self.track_knowledge.values)))
			

		#self.sensorScreen.update(carstate, self.rangeAngles)
		self.previousCarstate = carstate

		# self.hasHadSpeedAt = carstat
		command = self.unstuck(carstate,command)
		return command


	def startCsv (self):
		headers = ([
			"accel",
			"brake",
			"steer",
			"angle",
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
		self.csvFile = open('/home/student/cruise.csv', 'w')
		header = ';'.join(headers)
		self.csvFile.write(header + '\n')


	def writeCsv (self, carstate, command):
		rowData = []
		rowData.append(command.accelerator)
		rowData.append(command.brake)
		rowData.append(command.steering)
		rowData.append(carstate.angle)
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
		for thing in carstate.wheel_velocities:
			rowData.append(thing) 
		for thing in carstate.distances_from_edge:
			rowData.append(thing) 
		for thing in carstate.opponents:
			rowData.append(thing)
		row = ';'.join([str(thing) for thing in rowData])
		self.csvFile.write(row + '\n')