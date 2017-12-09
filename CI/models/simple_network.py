import os, sys, math
from sklearn.model_selection import train_test_split
import pandas as pd
import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
from torch.autograd import Variable as var
import matplotlib.pyplot as plt


def train_model():
    dataset = TorcsDataLoader.TorcsTrackDataset(['aalborg.csv', 'alpine-1.csv', 'f-speedway.csv'])
    x = var(torch.FloatTensor(dataset.carstates))
    y = var(torch.FloatTensor(dataset.targets), requires_grad=False)

    # x = var(torch.from_numpy(x_train).type(torch.FloatTensor), requires_grad=True)
    # y = var(torch.from_numpy(y_train).type(torch.FloatTensor), requires_grad=False)

    model = SimpleNetwork(dataset.x_dim, 13, 1)
    loss_fn = torch.nn.MSELoss(size_average=False)
    learning_rate = 1e-7

    for t in range(1000):
        # Forward pass: compute predicted y by passing x to the model. Module objects
        # override the __call__ operator so you can call them like functions. When
        # doing so you pass a Variable of input data to the Module and it produces
        # a Variable of output data.
        y_pred = model(x)

        # Compute and print loss. We pass Variables containing the predicted and true
        # values of y, and the loss function returns a Variable containing the
        # loss.
        loss = loss_fn(y_pred, y)
        # Zero the gradients before running the backward pass.
        model.zero_grad()

        # Backward pass: compute gradient of the loss with respect to all the learnable
        # parameters of the model. Internally, the parameters of each Module are stored
        # in Variables with requires_grad=True, so this call will compute gradients for
        # all learnable parameters in the model.
        loss.backward()

        # torch.nn.utils.clip_grad_norm(model.parameters(), 0.25)

        # Update the weights using gradient descent. Each parameter is a Variable, so
        # we can access its data and gradients like we did before.
        for param in model.parameters():
            param.data -= learning_rate * param.grad.data

        if t % 100 == 0:
            print("epoch", t, "total loss", loss.data)

    TorcsDataLoader.save_parameters(model, 'steering-all')
    return

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

if __name__ == '__main__':
    import TorcsDataLoader
    train_model()
