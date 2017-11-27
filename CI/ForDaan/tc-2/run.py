#! /usr/bin/env python3

from pytocl.main import main
from my_driver import MyDriver
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



if __name__ == '__main__':
    main(MyDriver(logdata=False))
