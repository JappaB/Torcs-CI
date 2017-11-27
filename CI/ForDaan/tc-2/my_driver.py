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
	input_size = 72
	hidden_size = 50
	output_size = 3

	path = os.path.join('models','simple_ff_model.pt')

	neural_net = torch.load(path)
	# neural_net = Net(input_size, hidden_size, output_size)
	# neural_net.load_state_dict(torch.load(path))

	return neural_net

def state_var(carstate):

	# Extract input_data
	curr_state = np.asarray([carstate.speed_x, carstate.distance_from_center, carstate.angle]+list(carstate.distances_from_edge)+list(carstate.opponents))
	# Turn  input state into Torch variable
	inp_data = Variable(torch.from_numpy(curr_state).float())
	return inp_data




class MyDriver(Driver):
	# Override the `drive` method to create your own driver
	...
	# def drive(self, carstate: State) -> Command:
	#     # Interesting stuff
	#     command = Command(...)
	#     return command

	def __init__(self,logdata = False):
		super().__init__(logdata)

		# Load model
		self.neural_net = load_model()		

	def drive(self, carstate)-> Command:
		"""
        Produces driving command in response to newly received car state.

        This is a dummy driving routine, very dumb and not really considering a
        lot of inputs. But it will get the car (if not disturbed by other
        drivers) successfully driven along the race track.
        """

		command = Command()


		model_outp = self.neural_net(state_var(carstate))
		print('output0',float(model_outp.data[0]))

		# command.accelerator = model_outp.data[0]
		# command.brake =model_outp.data[1]
		# command.steering = model_outp.data[2]
		self.steer(carstate, 0.0, command)
		# v_x = 80
		ACC_LATERAL_MAX = 6400 * 5
		v_x = min(80, math.sqrt(ACC_LATERAL_MAX / abs(command.steering)))

		print('steering command',command.steering)
		print('accelerator command', model_outp.data[0])
		print('brake command', model_outp.data[1])

		self.accelerate(carstate, v_x, command)

		return command

