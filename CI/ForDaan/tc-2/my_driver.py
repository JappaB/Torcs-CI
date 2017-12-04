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
        self.fc3 = nn.Linear(hidden_size, hidden_size)
        self.fc4 = nn.Linear(hidden_size, hidden_size)
        self.fc5 = nn.Linear(hidden_size, output_size)
    
    def forward(self, x):
        h = F.relu(self.fc1(x))
        h = F.relu(self.fc2(h))
        h = F.relu(self.fc3(h))
        h = F.relu(self.fc4(h))
        h = F.tanh(self.fc5(h))
        return h

    
class RNNJens(torch.nn.Module):
    def __init__(self, D_in, H, D_out):
        """
        In the constructor we instantiate two nn.Linear modules and assign them as
        member variables.
        """
        super(RNNJens, self).__init__()
        self.hidden_dim = H
        
        self.recurrent = nn.LSTM(D_in, H)
        self.linear = nn.Linear(H, D_out)
        self.hidden = self.init_hidden()
        
    def init_hidden(self):
        """
        Before we've done anything, we dont have any hidden state.
        Refer to the Pytorch documentation to see exactly
        why they have this dimensionality.
        The axes semantics are (num_layers, minibatch_size, hidden_dim)
        """
        return (torch.autograd.Variable(torch.zeros(1, 1, self.hidden_dim)),
                torch.autograd.Variable(torch.zeros(1, 1, self.hidden_dim)))
    
    def addNoise(self, noise):
        print(self.recurrent.weights.shape())
        
    def forward(self, x):
        """
        In the forward function we accept a Variable of input data and we must return
        a Variable of output data. We can use Modules defined in the constructor as
        well as arbitrary operators on Variables.
        """
        lstm_out, self.hidden = self.recurrent(x, self.hidden)
        y = self.linear(lstm_out)
        y_pred = F.sigmoid(y)
        
        return y_pred

class SimpleNetwork(nn.Module):
    def __init__(self, in_dim, hidden_units, out_dim):
        super(SimpleNetwork, self).__init__()
        self.in_dim = in_dim
        self.hidden_units = hidden_units
        self.out_dim = out_dim

        self.lin1 = torch.nn.Linear(in_dim, hidden_units)
        self.lin2 = torch.nn.Linear(hidden_units, out_dim)

    def forward(self, inputs):
        out = self.lin1(inputs)
        # out = F.sigmoid(out)
        out = self.lin2(out)
        out = F.tanh(out)
        return out

    def get_n_units(self):
        return (self.in_dim, self.hidden_units, self.out_dim)



class RNN2(nn.Module):
	def __init__(self, input_size, hidden_units, output_size):
	    super(RNN2, self).__init__()

	    self.hidden_units = hidden_units
	    self.in_dim = input_size
	    self.out_dim = output_size

	    # define layers
	    self.input = nn.Linear(input_size, hidden_units)
	    self.rnn = nn.LSTM(hidden_units, hidden_units, num_layers=2, dropout=0.05)
	    self.output = nn.Linear(hidden_units, output_size)

	def forward(self, input, hidden):
	    inp = self.input(input.view(1,-1)).unsqueeze(1)
	    output, hidden = self.rnn(inp, hidden)
	    output = self.output(output.view(1, -1))
	    return output, hidden

	def init_hidden(self, batch_size):
	    return (Variable(torch.zeros(2, batch_size, self.hidden_units)),
	                Variable(torch.zeros(2, batch_size, self.hidden_units)))

	def get_n_units(self):
	    return (self.in_dim, self.hidden_units, self.out_dim)

def load_model():
	#Hyper parameters
	input_size = 22
	hidden_size = 15
	output_size = 1

	path = os.path.join('models','steering_22-15-1_RNN.h5')

	#neural_net = torch.load(path)
	neural_net = RNN2(input_size, hidden_size, output_size)
	hidden = neural_net.init_hidden(1)
	neural_net.load_state_dict(torch.load(path))

	return neural_net

def state_var(carstate):
# 								speedX	speedY	speedZ	trackPos	z	wheelSpinVel01	wheelSpinVel02	wheelSpinVel03	wheelSpinVel04	track00	track18	oppos00	oppos35

	# Extract input_data
	curr_state = np.asarray([
		# carstate.speed_x, 
		# carstate.speed_y, 
		# carstate.speed_z ,
		carstate.distance_from_center, 
		carstate.angle]
		# carstate.z]
		+list(carstate.distances_from_edge)
		# +list(carstate.opponents)
		)
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
		#self.startCsv()

		#ant colony feromone knowledge drop
		self.track_knowledge = {}
		self.total_angle_prev = 0.0

	def on_shutdown(self):
		#self.csvFile.close()
		...

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
		"""
		Produces driving command in response to newly received car state.

		This is a dummy driving routine, very dumb and not really considering a
		lot of inputs. But it will get the car (if not disturbed by other
		drivers) successfully driven along the race track.
		"""
		command = self.cruise(carstate)

		self.drop_knowledge(carstate, command)
		# print('angle',carstate.angle)
		# print('steering converted', ((math.pi*command.steering)/180.0))
		#self.writeCsv(carstate, command)
		# command = Command()
		# state = state_var(carstate)


		# net = load_model()
		# hidden = net.init_hidden(1)
		# model_outp = net(state,hidden)
		# print(model_outp)

		# v_x = 0

		# command.accelerator = v_x
		# command.steering = model_outp[0].data[0]* 0.5
		# print(command.steering)
		#print(model_outp.data[0])
		#if carstate.current_lap_time > 10:
		# command.steering = model_outp.data[0]


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
		# carstate.angle *= math.pi/180.0
		self.sensorScreen.update(carstate, self.rangeAngles)
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