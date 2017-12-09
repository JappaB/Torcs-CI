import torch
from nnet_structure import DriverNet
import os

if __name__ == '__main__':
	path = os.path.join('evonets', 'pool2')
	names= ['Anne', 'Bart', 'Carl', 'Daan', 'Earl', 'Fred', 
	'Greg', 'Hans', 'Iris', 'Jens', 'Kent', 'Lois', 'Mark', 
	'Nora', 'Olga', 'Pete', 'Qurl', 'Rick', 'Stan', 'Theo', 
	'Uris', 'Vlad', 'Walt', 'Xavi', 'York', 'Zack']

	for name in names:
		DriverNet.CreateRandom(path, name)

		# nets = DriverNet.CreateRandom(path, name)
		# for i in range(100):
		# 	nets['steer'].addNoise(1.0)
		# name = "{}_0_{:.6f}".format(name, 1.0)
		# DriverNet.saveNetSet(nets, path, name)