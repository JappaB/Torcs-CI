import torch
from nnet_structure import DriverNet
import os

if __name__ == '__main__':
	# load neural net files
	path = os.path.join('evonets', 'pool1')
	nnetNames = ['Anne_0.pt', 'Bart_0.pt', 'Carl_0.pt', 'Daan_0.pt', 'Earl_0.pt', 'Fred_0.pt', 
	'Greg_0.pt', 'Hans_0.pt', 'Iris_0.pt', 'Jens_0.pt', 'Kent_0.pt', 'Lois_0.pt', 'Mark_0.pt', 
	'Nora_0.pt', 'Olga_0.pt', 'Pete_0.pt', 'Qurl_0.pt', 'Rick_0.pt', 'Stan_0.pt', 'Theo_0.pt', 
	'Uris_0.pt', 'Vlad_0.pt', 'Walt_0.pt', 'Xavi_0.pt', 'York_0.pt', 'Zack_0.pt']

	for nnetName in nnetNames:
		nnet = DriverNet()
		torch.save(nnet.state_dict(), os.path.join(path, nnetName))
