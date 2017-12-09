import os
from evoManager import filterByValue
from nnet_structure import DriverNet
import torch

if __name__ == '__main__':
	# nnets = os.listdir('evonets/pool2')
	# nnets = filterByValue(nnets, '.pt')

	netsToCopy = DriverNet.loadNetSet('evonets/pool1', 'Hans_58_0.658654')


	names= ['Anne', 'Bart', 'Carl', 'Daan', 'Earl', 'Fred', 
		'Greg', 'Hans', 'Iris', 'Jens', 'Kent', 'Lois', 'Mark', 
		'Nora', 'Olga', 'Pete', 'Qurl', 'Rick', 'Stan', 'Theo', 
		'Uris', 'Vlad', 'Walt', 'Xavi', 'York', 'Zack']
	noise = 0.0
	for name in names:
		DriverNet.saveNetSet(netsToCopy, 'evonets/pool3', name+"_0_{:.6f}".format(noise))
		#netsToCopy['speed'].addNoise(0.1)
		netsToCopy['steer'].addNoise(0.05)
		noise += 1.0
