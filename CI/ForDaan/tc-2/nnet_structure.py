import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import numpy as np


class DriverNet (torch.nn.Module):
    def __init__(self):
        super(DriverNet, self).__init__()
        self.inputDimension = 61
        self.hiddenDimension = 50
        self.outputDimension = 3
        
        self.recurrent = nn.LSTM(self.inputDimension, self.hiddenDimension)
        self.linear = nn.Linear(self.hiddenDimension, self.outputDimension)
        self.hidden = self.init_hidden()
        
    def init_hidden(self):
        return (torch.autograd.Variable(torch.zeros(1, 1, self.hiddenDimension)),
                torch.autograd.Variable(torch.zeros(1, 1, self.hiddenDimension)))
    
    def addNoise (self, noiseFactor):
        #print(next(self.parameters()).data[0,0])
        for weightTensor in self.parameters():
            shape = weightTensor.size()
            randoms = torch.randn(*shape)
            randoms *= noiseFactor
            weightTensor.data += randoms
        #print(next(self.parameters()).data[0,0])

    def forward(self, x):
        lstm_out, self.hidden = self.recurrent(x.view(1,1,-1), self.hidden)
        y = self.linear(lstm_out)
        y_pred = F.tanh(y)
        
        return y_pred