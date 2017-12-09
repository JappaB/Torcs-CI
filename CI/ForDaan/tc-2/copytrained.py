import os
from evoManager import filterByValue
from nnet_structure import DriverNet
import torch

if __name__ == '__main__':
	nnets = os.listdir('evonets/pool1')
	nnets = filterByValue(nnets, 'speed.pt')


	netToCopy = DriverNet('steer')
	netToCopy.load_state_dict(torch.load('evonets/trained/net_cruise_to_win_2000epc.speed.pt'))

	for nnet in nnets:
		torch.save(netToCopy.state_dict(), os.path.join('evonets/pool1', nnet))
		#netToCopy.addNoise(0.1)
