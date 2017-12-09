import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import numpy as np
import os


class DriverNet (torch.nn.Module):
	variants = ['speed', 'steer']

	def __init__(self, variant):
		super(DriverNet, self).__init__()
		self.variant = variant
		self.inputDimension = 21
		self.hiddenDimension = 5
		self.outputDimension = 1
		self.numberOfLayers = 2
		
		self.lstm = nn.LSTM(
			input_size=self.inputDimension, 
			hidden_size=self.hiddenDimension,
			num_layers=self.numberOfLayers
			)

		self.hidden = self.init_hidden()
		self.linear = nn.Linear(self.hiddenDimension, self.outputDimension)

		# for weightTensor in self.parameters():
		# 	weightTensor.data = torch.zeros(weightTensor.size())
			
	def init_hidden(self):
		hidden1 = torch.zeros(self.numberOfLayers, 1, self.hiddenDimension)
		hidden2 = torch.zeros(self.numberOfLayers, 1, self.hiddenDimension)
		# if torch.cuda.is_available():
		# 	hidden1, hidden2 = hidden1.cuda(), hidden2.cuda()
		return (Variable(hidden1),Variable(hidden2))
	
	def addNoise (self, noiseFactor):
		for weightTensor in self.parameters():
			shape = weightTensor.size()
			noise = torch.randn(*shape)
			noise *= noiseFactor
			# only affect 10% of weights on average:
			# scale [0,1) range such that 90% will fall 
			# under 0.5, and then round to integer and 
			# use element-wise multiplication of noise
			# yep, made it up myself
			fractionAffected = 0.1
			noiseFilter = torch.rand(*shape)
			noiseFilter /= (1-fractionAffected) / 0.5
			noiseFilter = torch.round(noiseFilter)
			noise *= noiseFilter
			# add noise to weights
			weightTensor.data += noise

	def forward(self, x):
		lstmOut, self.hidden = self.lstm(x.view(1,1,-1), self.hidden)
		linearOut = self.linear(lstmOut)
		linearOut = F.tanh(linearOut)
		return linearOut

	def CreateRandom (path, name):
		for variant in DriverNet.variants:
			nnet = DriverNet(variant)
			# if torch.cuda.is_available():
			# 	nnet = nnet.cuda()
			fileName = "{}_0_{:.6f}.{}.pt".format(name, 1.0, variant)
			torch.save(nnet.state_dict(), os.path.join(path, fileName))
		name = "{}_0_{:.6f}".format(name, 1.0)
		return DriverNet.loadNetSet(path, name)

	def loadNetSet (path, name):
		nnets = {}
		for variant in DriverNet.variants:
			fileName = "{}.{}.pt".format(name, variant)
			filePath = os.path.join(path, fileName)
			nnet = DriverNet(variant)
			# if torch.cuda.is_available():
			# 	nnet = nnet.cuda()
			nnet.load_state_dict(torch.load(filePath))
			nnets[variant] = nnet
		return nnets

	def saveNetSet (nnets, path, name):
		for variant, nnet in nnets.items():
			fileName = "{}.{}.pt".format(name, variant)
			torch.save(nnet.state_dict(), os.path.join(path, fileName))

	def removeNetSet (path, name):
		for variant in DriverNet.variants:
			fileName = "{}.{}.pt".format(name, variant)
			os.remove(os.path.join(path, fileName))
