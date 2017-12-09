import torch
import torch.nn as nn
from torch.nn import functional as F
from torch.autograd import Variable as var
from torch import optim

from sklearn.model_selection import train_test_split
import pandas as pd
import numpy as np
import math, random, sys, os


class RNN(nn.Module):
    def __init__(self, input_size, hidden_units, output_size):
        super(RNN, self).__init__()

        self.hidden_units = hidden_units
        self.in_dim = input_size
        self.out_dim = output_size

        # define layers
        self.input = nn.Linear(input_size, hidden_units)
        self.rnn = nn.LSTM(hidden_units, hidden_units, num_layers=2, dropout=0.05)
        self.output = nn.Linear(hidden_units, output_size)

    # # internal version of forward (i think)
    # def step(self, input, hidden=None):
    #     trans_input = self.input(input.view(1, -1)).unsqueeze(1)
    #     out, hidden = self.rnn(trans_input, hidden)
    #     out = self.output(out.squeeze(1))
    #     return out, hidden

    # def forward(self, inputs, hidden=None, force=True, steps=0):
    #     if force or steps == 0:
    #         steps = len(inputs)
    #
    #     # initialize buffer for memory
    #     outputs = var(torch.zeros(steps, 1, 1))
    #
    #     # iterate over lenght of inputs (each inp is 1xinput_size)
    #     for i in range(steps):
    #         if force or i == 0:
    #             inp = inputs[i]
    #         else:
    #             # here you replace the input by the previous output
    #             inp = output
    #
    #         # do regular forward pass based on current input
    #         output, hidden = self.step(inp, hidden)
    #
    #         # add ouput to mempory
    #         outputs[i] = output
    #
    #     return outputs, hidden

    def forward(self, input, hidden):
        inp = self.input(input.view(1,-1)).unsqueeze(1)
        output, hidden = self.rnn(inp, hidden)
        output = self.output(output.view(1, -1))
        return output, hidden

    def init_hidden(self, batch_size):
        return (var(torch.zeros(2, batch_size, self.hidden_units)),
                    var(torch.zeros(2, batch_size, self.hidden_units)))

    def get_n_units(self):
        return (self.in_dim, self.hidden_units, self.out_dim)

def train_rnn():
    n_epochs = 5
    hidden_units = 15

    data = TorcsDataLoader.TorcsTrackDataset(['aalborg.csv',
                                              'alpine-1.csv',
                                              'f-speedway.csv'])

    model = RNN(data.x_dim, hidden_units, 1)
    loss_func = nn.MSELoss()
    optimzer = optim.Adam(model.parameters(), lr=1e-4)



    for epoch in range(n_epochs):
        total_loss = 0
        for i, observation in enumerate(data):
            x_t = observation['input']
            target = observation['target']

            if i % 1000 == 0: print(i)

            hidden = model.init_hidden(1)

            inp = var(torch.FloatTensor(x_t))
            target = var(torch.FloatTensor([target]), requires_grad=False)

            outputs, hidden = model(inp, hidden)
            model.zero_grad()

            loss = loss_func(outputs.view(1, -1), target)
            loss.backward()
            optimzer.step()

            total_loss += loss.data[0]

        if epoch % 10 == 0:
            print('Epoch', epoch, "Loss", total_loss)

    TorcsDataLoader.save_parameters(model, 'steering')

if __name__ == '__main__':
    import TorcsDataLoader
    train_rnn()
