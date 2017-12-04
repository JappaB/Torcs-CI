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


def load_model(path):
	#path = os.path.join('models','teringdict.pt')
	neural_net = DriverNet()
	neural_net.load_state_dict(torch.load(path))

	return neural_net


def state_var(carstate):
	# Extract input_data
	curr_state = np.asarray([
		carstate.angle,
		carstate.speed_x / (200.0/3.6), 
		carstate.speed_y, 
		carstate.speed_z ,
		carstate.distance_from_center, 
		carstate.z]
		+[bla / (200.0) for bla in list(carstate.distances_from_edge)
		+[bla / (200.0) for bla in list(carstate.opponents)]
		])
	# Turn  input state into Torch variable
	inp_data = Variable(torch.from_numpy(curr_state).float())
	return inp_data




class MyDriver(Driver):

	def __init__(self, nnetPath, logdata = False):
		super().__init__(logdata)
		self.yeaIFuckedUp = False
		self.lastMoved = 0.0
		self.backwardDistance = 0.0
		self.previousCarstate = 0
		self.topSpeed = 0.0
		self.performance = 0.0
		# default angles, will be overwritten by range_finder_angles()
		self.rangeAngles = [-90, -65, -50, -35, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 35, 50, 65, 90]
		# Load model
		self.neural_net = load_model(nnetPath)
		# open sensor data viewer
		self.sensorScreen = SensorScreen()
		# remembering gas and brake pedals
		self.accelPressure = 1.0
		self.brakePressure = 0.2
		#self.startCsv()

		#ant colony feromone knowledge drop
		self.track_knowledge = {}
		self.total_angle_prev = 0.0

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

	@property
	def range_finder_angles(self):
		return self.rangeAngles
		self.rangeAngles = []
		roadWidth = 11
		sensorFocus = 25
		sampleSpread = (np.logspace(0, 1.0, base=sensorFocus, num=9) - 1)/(sensorFocus-1)
		for i in range(19):
			if i < 9:
				angle = np.arctan((sampleSpread[i]*200.0) / (roadWidth/2))
				angle = ((-math.pi/2.0) + angle) * 180/math.pi
			elif i == 9:
				angle = 0
			elif i > 9:
				angle = np.arctan((sampleSpread[(9-(i-9))]*200.0) / (roadWidth/2))
				angle = ((math.pi/2.0) - angle) * 180/math.pi
			self.rangeAngles.append(angle)
		return self.rangeAngles


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
		command.steering = (
				self.steerToCorner(carstate.angle) + 
				self.steerToCenter(carstate.distance_from_center) + 
				self.steerToFurthestRange(carstate.distances_from_edge)+
				self.heat_avoiding_missile(list(carstate.opponents)))

		#dampen steering PD-controller
		if hasattr(MyDriver, 'steering_prev'):
			#diff
			delta_steer = steering_prev-command.steering
			command.steering -= delta_steer*0.5
			self.steering_prev = command.steering
		else:
			self.steering_prev = command.steering




		#TODO faster cruise 
		command.gear = autoTransmission(carstate.gear, carstate.rpm, 0.0)
		targetSpeed = 80 # kph
		targetSpeed -= 100 * abs(command.steering)
		targetSpeed -= (100 - np.max(carstate.distances_from_edge))/2
		targetSpeed = max(targetSpeed, 30)
		targetSpeed /= 3.6 # m/s
		self.accelPressure += (targetSpeed - carstate.speed_x) * 0.01
		self.accelPressure = np.clip(self.accelPressure, 0, 0.6)
		command.accelerator = self.accelPressure
		command.brake = 0.0
		if carstate.speed_x > 10 and carstate.speed_x*2 > np.max(carstate.distances_from_edge):
			command.brake = 0.5
			command.accelerator = 0.0

		# unstucking (WIP)
		# isStuck = (
		# 		carstate.distances_from_edge[0] == -1 
		# 		or abs(carstate.distance_from_center) > 0.8)
		# if isStuck:
		# 	print("arg {:.2f}".format(carstate.current_lap_time))
		# 	command.accelerator = 0.2
		# 	command.steering = (
		# 			self.steerToCorner(carstate.angle) + 
		# 			self.steerToCenter(carstate.distance_from_center))
		# 	if carstate.speed_x < 0.05:
		# 		if int(carstate.current_lap_time/5)%2 ==0:
		# 			command.gear = 1
		# 		else:
		# 			command.gear = -1
		# 			command.steering *= -1
		# elif carstate.gear < 0:
		# 	command.gear = 1
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


	def drop_knowledge(self, carstate, command):
		# {dist from start (int): angle of steering + angle(bycicle model),....}

		self.total_angle = carstate.angle+((math.pi*command.steering)/180.0)
		# Interpolation TODO?
		print('dist raced', carstate.distance_from_start)
		#drop knowledge every 5m (TODO: Only if you are front car which drives savely)
		if (int(carstate.distance_from_start)) and (carstate.last_lap_time ==0):
			self.total_angle_prev = self.total_angle
			self.track_knowledge[int(carstate.distance_from_start)] = self.total_angle
			# print('lap1',command.steering)
		# second round first car starts using the knowledge as well
		elif(carstate.last_lap_time != 0):
			print('lookup steering angle last round:')
			print('lookup',(self.track_knowledge[str(int(carstate.distance_from_start))]
				*180/math.pi)

			command.steering = (
				self.steerToCenter(carstate.distance_from_center) +
				(self.track_knowledge[str(int(carstate.distance_from_start))]*180/(math.pi)))
			print('lap2',command.steering)


		# print('prev_lap_time',carstate.last_lap_time) 
		# print(self.track_knowledge)


	def drive(self, carstate)-> Command:
		if self.previousCarstate == 0:
			self.previousCarstate = carstate
		command = Command()

		# In evolution mode:
		# Race casually after you fucked up
		if self.yeaIFuckedUp:
			return command
			#return self.cruise(carstate)

		self.drop_knowledge(carstate, command)
		# for now, drive casually always
		#command = self.cruise(carstate)

		# drive with neural net
		nnetOut = self.neural_net(state_var(carstate))[0][0]
		command.accelerator = nnetOut.data[0]
		command.brake = nnetOut.data[1]
		command.steering = nnetOut.data[2]

		# simplify controls to train faster
		frontSlip, rearSlip = calcWheelSlip(carstate.speed_x, carstate.wheel_velocities)
		command.gear = autoTransmission(carstate.gear, carstate.rpm, rearSlip)
		if command.accelerator > 0.5:
			# step on the fucking gas
			if carstate.speed_x > 2:
				self.accelPressure = accelTcl(self.accelPressure, rearSlip)
			command.accelerator = self.accelPressure

		if command.brake > 0.5:
			# slam on the brakes, no gas
			self.brakePressure = brakeAbs(self.brakePressure, frontSlip)
			command.brake = self.brakePressure
			command.accelerator = 0.0
		else:
			command.brake = 0.0


		# Retire when you fucked up. When half of the 
		# participants have fucked up, the race ends.
		self.performance = carstate.distance_raced
		if self.hasFuckedUp(carstate, self.previousCarstate):
			command.meta = 2
			self.yeaIFuckedUp = True
			if carstate.distance_raced > 100 and carstate.distance_raced < 3000:
				avgSpeed = carstate.distance_raced / carstate.current_lap_time # add early incentive for speed
				speedBonus = min(avgSpeed, 60.0) # passing startfinish can fuck it up
				crashSpeedPenalty = carstate.speed_x * 3.6 # in kph for extra negative incentive
				brakingBonus = crashSpeedPenalty * (1.0 - command.brake) * 0.5 # give some slack when he was at least braking
				self.performance += speedBonus
				self.performance -= crashSpeedPenalty
				self.performance += brakingBonus
				self.performance = max(self.performance, 1.0) # prevent being randomized


		#self.writeCsv(carstate, command)
		#print('output0',float(model_outp.data[0]))
		command.gear = autoTransmission(carstate.gear,carstate.rpm,0)
		#print('accel: {} brake: {} steer: {}'.format(model_outp.data[0], model_outp.data[1], model_outp.data[2]))

		# brake = model_outp.data[1]
		# if brake > 0.1:
		# 	command.accelerator = 0.0
		# 	command.brake = brake
		# else:
		# 	command.accelerator = model_outp.data[0]
		#command.accelerator = model_outp.data[0]
		#command.brake = 0.0
		#if model_outp.data[1] > 0.3:
		#	command.brake = model_outp.data[1]
		#	command.accelerator = 0.0
		#command.steering = (model_outp.data[2] - 0.5) * 2.0
		# self.steer(carstate, 0.0, command)
		# v_x = 80
		# ACC_LATERAL_MAX = 6400 * 5
		# v_x = min(80, math.sqrt(ACC_LATERAL_MAX / abs(command.steering)))

		# print('steering command',command.accelerator)
		# print('accelerator command',command.accelerator)
		# print('brake command', model_outp.data[1])


		# print('steering command',command.steering)
		# print('accelerator command', model_outp.data[0])
		# print('brake command', model_outp.data[1])

		# self.accelerate(carstate, v_x, command)
		#self.sensorScreen.update(carstate, self.rangeAngles)
		self.previousCarstate = carstate
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