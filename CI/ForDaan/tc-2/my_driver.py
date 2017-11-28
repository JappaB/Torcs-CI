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

	return gear

# Neural Network Model
class Net(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, output_size)
    
    def forward(self, x):
        h = F.tanh(self.fc1(x))
        h = F.tanh(self.fc2(h))
        h = F.tanh(self.fc3(h))
        return h



def load_model():
	#Hyper parameters
	input_size = 61
	hidden_size = 50
	output_size = 2

	path = os.path.join('models','simple_ff_model_5epochs_batch100_dict_hiden_size50_2HL_INP72_outp3_Forward_tanh_tanh_tanh_onlyrelevantdata.pt')

	neural_net = torch.load(path)
	# neural_net = Net(input_size, hidden_size, output_size)
	# neural_net.load_state_dict(torch.load(path))

	return neural_net

def state_var(carstate):
# 								speedX	speedY	speedZ	trackPos	z	wheelSpinVel01	wheelSpinVel02	wheelSpinVel03	wheelSpinVel04	track00	track18	oppos00	oppos35

	# Extract input_data
	curr_state = np.asarray([carstate.angle, carstate.current_lap_time, carstate.distance_from_start, carstate.distance_raced,carstate.gear,carstate.last_lap_time,carstate.race_position,carstate.rpm,carstate.speed_x , carstate.speed_y, carstate.speed_z ,carstate.distance_from_center, carstate.z]+list(carstate.wheel_velocities)+list(carstate.distances_from_edge)+list(carstate.opponents))
	# Turn  input state into Torch variable
	inp_data = Variable(torch.from_numpy(curr_state).float())
	return inp_data




class MyDriver(Driver):

	def __init__(self,logdata = False):
		super().__init__(logdata)
		# default angles, will be overwritten by range_finder_angles()
		self.rangeAngles = [-90, -65, -50, -35, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 35, 50, 65, 90]
		# Load model
		self.neural_net = load_model()
		# open sensor data viewer
		self.sensorScreen = SensorScreen()
		# remembering gas and brake pedals
		self.accelPressure = 1.0
		self.brakePressure = 0.2
	
	def on_shutdown(self):
		...

	@property
	def range_finder_angles(self):
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
				self.steerToFurthestRange(carstate.distances_from_edge))
		command.gear = autoTransmission(carstate.gear, carstate.rpm, 0.0)
		targetSpeed = 100 # kph
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


	def drive(self, carstate)-> Command:
		"""
		Produces driving command in response to newly received car state.

		This is a dummy driving routine, very dumb and not really considering a
		lot of inputs. But it will get the car (if not disturbed by other
		drivers) successfully driven along the race track.
		"""

		# command = self.cruise(carstate)
		command =Command()

		model_outp = self.neural_net(state_var(carstate))
		# print('output0',float(model_outp.data[0]))
		# command.gear = autoTransmission(carstate.gear,carstate.rpm,0)
		print('accel/brake',model_outp.data[0])

		if model_outp.data[0] >= 0:
			command.accelerator = model_outp.data[0]

		if model_outp.data[0]<0:
			command.brake = model_outp.data[0]

		command.steering = model_outp.data[1]
		# self.steer(carstate, 0.0, command)
		# v_x = 80
		# ACC_LATERAL_MAX = 6400 * 5
		# v_x = min(80, math.sqrt(ACC_LATERAL_MAX / abs(command.steering)))

		print('steering command',command.accelerator)
		print('accelerator command',command.accelerator)
		print('brake command', model_outp.data[1])


		# print('steering command',command.steering)
		# print('accelerator command', model_outp.data[0])
		# print('brake command', model_outp.data[1])

		# self.accelerate(carstate, v_x, command)
		self.sensorScreen.update(carstate, self.rangeAngles)
		return command

