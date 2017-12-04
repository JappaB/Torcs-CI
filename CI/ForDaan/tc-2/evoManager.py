from multiprocessing import Process, Queue
from my_driver import MyDriver
from pytocl.protocol import Client
import os
from random import shuffle
import torch
import re
from nnet_structure import DriverNet
import random as r
import math
from collections import defaultdict

def runDriver (startPos, portNumber, path, nnetName, q):
	# start client loop:
	nnetPath = os.path.join(path, nnetName)
	driver = MyDriver(nnetPath, logdata=False)
	client = Client(driver=driver, port=portNumber, hostname='localhost')
	score = client.run()
	q.put([startPos, nnetName, score])


def loadNnet (filename):
	path = os.path.join('evonets', 'pool1', filename)
	nnet = DriverNet()
	nnet.load_state_dict(torch.load(path))
	return nnet

def getChildName (path, parentName, noiseFactor):
	parentGeneration = int(re.search(r'\d+', parentName).group())
	childGeneration = parentGeneration + 1
	dstPathfmt = "%s%d%s"
	parts = os.path.splitext(parentName)
	childName = parts[0][:5] + str(childGeneration) + "_{:.6f}".format(noiseFactor) + ".pt"
	while os.path.exists(os.path.join(path, childName)):
		noiseFactor += 0.000001
		childName = parts[0][:5] + str(childGeneration) + "_{:.6f}".format(noiseFactor) + ".pt"
	return childName

if __name__ == '__main__':
	q = Queue()
	leaderboard = defaultdict(lambda: 0.0)

	while True:
		# load neural net files
		path = os.path.join('evonets', 'pool1')
		nnetNames = os.listdir(path)
		if len(nnetNames) < 10:
			print("There's fewer than 10 nnet files in {}".format(path))
			break

		# remove losers from scoreboard

		# remove loser from leaderboard
		losers = set(list(leaderboard.keys())) - set(nnetNames)
		for loser in losers:
			del leaderboard[loser]

		# randomly shuffle for selecting 10
		# participants and determining starting grid
		shuffle(nnetNames)

		print("----- new race -----")
		# start drivers
		processes = []
		for i in range(10):
			port = 3001 + i
			startPos = i+1
			p = Process(target=runDriver, args=(startPos, port, path, nnetNames[i], q))
			processes.append(p)
			p.start()

		# wait for race to end
		for process in processes:
			process.join()

		# prepare scoreboard
		unsortedScoreboard = {}
		while not q.empty():
			startPos, nnet, score = q.get()
			unsortedScoreboard[nnet] = score
		scoreboard = sorted(unsortedScoreboard.items(), key=lambda x:x[1], reverse=True)

		# save results

		# replace drivers that didn't move at all with random net
		replacedDrivers = []
		for nnetName, score in scoreboard:
			if score < 1:
				randomNnet = DriverNet()
				torch.save(randomNnet.state_dict(), os.path.join(path, nnetName))
				replacedDrivers.append(nnetName)

		# determine winner and loser
		winningScore = scoreboard[0][1]
		winner = scoreboard[0][0]
		loser = scoreboard[-1][0]

		if winningScore > 1.0:
			# load winning net
			nnet = loadNnet(winner)

			# add noise to net
			noiseFactor = r.random() * 1.1
			noiseFactor *= noiseFactor * noiseFactor * noiseFactor
			nnet.addNoise(noiseFactor)

			# give it a new name
			childName = getChildName(path, winner, noiseFactor)

			# add net to pool
			torch.save(nnet.state_dict(), os.path.join(path, childName))

			# remove loser from pool
			os.remove(os.path.join(path, loser))

		# print results
		for nnetName, score in scoreboard:
			note = ""
			if nnetName in replacedDrivers:
				note = "replaced with random"
			if winningScore > 1.0:
				if nnetName == winner:
					note = "birthed {}".format(childName)
				if nnetName == loser:
					note = "RIP"
			if math.isnan(score):
				score = "???"
			else:
				leaderboard[nnetName] += score
				score = int(score)

			print("{}: \t\t{} \t\t{}".format(nnetName, score, note))

		# print the current leaders
		print("top dogs:")
		hasTopDogDied = False
		topDogs = sorted(leaderboard.items(), key=lambda x:x[1], reverse=True)[:3]
		for nnetName, score in topDogs:
			print("\t{}: \t\t{}".format(nnetName, int(score)))
			if nnetName == loser:
				hasTopDogDied = True
		
		# fun
		if hasTopDogDied:
			print("\t\t{}".format('*'.join(["RIP" for i in range(24)])))
			print("\t\t\tLet us remember the legendary \n\t\t\t\t\t\t\t{}, \n\t\t\t\t\t\t\t\t\twho perished from sheer misfortune.".format(loser))
			print("\t\t{}".format('*'.join(["RIP" for i in range(24)])))
