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
import pickle
import time
import subprocess
import argparse
import signal

def runServer (gui=False):
	command = "torcs -nofuel -r /home/daan/Torcs-CI/aitrain.xml"
	if gui:
		command = "torcs -nofuel -nodamage"
	return subprocess.Popen(command, shell=True)

def runDriver (startPos, portNumber, path, contestant, q):
	# start client loop:
	driver = MyDriver(path, contestant, logdata=False)
	client = Client(driver=driver, port=portNumber, hostname='localhost')
	score = client.run()
	q.put([startPos, contestant, score])

def filterByValue (sequence, value):
	for thing in sequence:
		if value in thing:
			yield thing

def loadContestants (path):
	contestants = os.listdir(path)
	contestants = filterByValue(contestants, '.pt')
	contestants = filterByValue(contestants, 'steer')
	contestants = [contestant[:-9] for contestant in contestants] # strip extension
	return contestants

def getChildName (path, parentName, noiseFactor):
	parentGeneration = int(re.search(r'\d+', parentName).group())
	childGeneration = parentGeneration + 1
	dstPathfmt = "%s%d%s"
	parts = os.path.splitext(parentName)
	childName = parts[0][:5] + str(childGeneration) + "_{:.6f}".format(noiseFactor)
	while os.path.exists(os.path.join(path, "{}.steer.pt".format(childName))):
		noiseFactor += 0.000001 # close enough
		childName = parts[0][:5] + str(childGeneration) + "_{:.6f}".format(noiseFactor)
	return childName

# pickle needs this to save scoreboard
def returnZero ():
	return 0.0

if __name__ == '__main__':

	evolveSpeed = False
	evolveSteer = True
	justChilling = False
	nrDrivers = 10
	poolPath = os.path.join('evonets', 'pool3')




	parser = argparse.ArgumentParser(
		description='evoclient'
	)
	parser.add_argument(
		'--gui',
		help='Start with GUI or not.',
		action='store_true',
		default=False
	)
	args = parser.parse_args()

	# start torcs server
	server = runServer(args.gui)

	q = Queue()


	# load leaderboard
	leaderboard = defaultdict(returnZero)
	leaderboardPath = os.path.join(poolPath, '1_leaderboard.pkl')
	if os.path.exists(leaderboardPath):
		with open(leaderboardPath, 'rb') as file:
			leaderboard = pickle.load(file)


	# load stats
	stats = {'raceCount': 0}
	statsPath = os.path.join(poolPath, '1_stats.pkl')
	if os.path.exists(statsPath):
		with open(statsPath, 'rb') as file:
			stats = pickle.load(file)


	# create scores file if it does not exist yet
	scoresPath = os.path.join(poolPath, '1_scores.csv')
	if not os.path.exists(scoresPath):
		headers = ["raceNumber", "time", "contestant", "score", "child"]
		header = ';'.join(headers)
		csvFile = open(scoresPath, 'w')
		csvFile.write(header + '\n')
		csvFile.close()


	while True:
		# load neural net files
		contestants = loadContestants(poolPath)
		if len(contestants) < 10:
			print("There's fewer than 10 contestants {}".format(path))
			break


		# remove losers from scoreboard
		losers = set(list(leaderboard.keys())) - set(contestants)
		for loser in losers:
			del leaderboard[loser]


		# save leaderboard
		with open(leaderboardPath, 'wb+') as file:
			pickle.dump(leaderboard, file)


		# randomly shuffle for selecting 10 participants
		# and for determining starting grid
		shuffle(contestants)


		# start drivers
		print("----- race #{} -----".format(stats['raceCount']+1))
		processes = []
		for i in range(nrDrivers):
			port = 3001 + i
			startPos = i+1
			p = Process(target=runDriver, args=(startPos, port, poolPath, contestants[i], q))
			processes.append(p)
			p.start()


		# wait for race to end
		abort = False
		for process in processes:
			if abort:
				process.terminate()
			else:
				timeout = 20.0
				if args.gui:
					timeout = 999.0
				process.join(timeout)
				if process.is_alive():
					abort = True
		if abort:
			print("Something went wrong, let's try again.")
			for process in processes:
				process.terminate()
			server.terminate()
			os.kill(server.pid, signal.SIGINT)
			subprocess.Popen("echo daan | sudo -S pkill -9 torcs-bin", shell=True)
			time.sleep(1)
			server = runServer(args.gui)
			continue

		stats['raceCount'] += 1


		# prepare scoreboard
		unsortedScoreboard = {}
		while not q.empty():
			startPos, nnet, score = q.get()
			unsortedScoreboard[nnet] = score
		scoreboard = sorted(unsortedScoreboard.items(), key=lambda x:x[1], reverse=True)


		# determine winner and loser
		winningScore = scoreboard[0][1]
		winner = scoreboard[0][0]
		loser = scoreboard[-1][0]


		# # replace drivers that didn't move at all with random net
		# replacedDrivers = []
		# for contestant, score in scoreboard:
		# 	if score < 1:
		# 		DriverNet.CreateRandom(poolPath, contestant)
		# 		replacedDrivers.append(contestant)
		childName = "none"
		if winningScore > 1.0:
			# load winning nets
			nnets = DriverNet.loadNetSet(poolPath, winner)

			# add noise to nets
			median = 0.3
			lamb = math.log(2.0)/median
			u = r.random()
			noiseFactor = -math.log(1-u**2)/lamb
			#print("random sample {:.2f} from exp distribution (log10) with median {:.3f}: {}".format(u, -math.log10(0.5)/lamb, noiseFactor))

			if evolveSpeed:
				nnets['speed'].addNoise(noiseFactor)
			if evolveSteer:
				nnets['steer'].addNoise(noiseFactor)

			# give it a new name
			childName = getChildName(poolPath, winner, noiseFactor)

			if not justChilling:
				# add net to pool
				DriverNet.saveNetSet(nnets, poolPath, childName)

				# remove loser from pool
				DriverNet.removeNetSet(poolPath, loser)


		# save results
		raceTime = time.time()
		for contestant, score in scoreboard:
			rowData = []
			rowData.append(stats['raceCount'])
			rowData.append(raceTime)
			rowData.append(contestant)
			rowData.append(score)
			child = ""
			if contestant == winner:
				child = childName
			elif contestant == loser:
				child = "RIP"
			rowData.append(child)
			row = ';'.join([str(thing) for thing in rowData])
			csvFile = open(scoresPath, 'a')
			csvFile.write(row + '\n')
			csvFile.close()


		# print results
		for contestant, score in scoreboard:
			note = ""
			# if contestant in replacedDrivers:
			# 	note = "replaced with random"
			if winningScore > 1.0:
				if contestant == winner:
					note = "birthed {}".format(childName)
				if contestant == loser:
					note = "RIP"
			if math.isnan(score): # apparently happens sometimes
				score = "???"
			else:
				leaderboard[contestant] += score
				score = int(score)

			print("{}: \t\t{} \t\t{}".format(contestant, score, note))


		# print the current leaders
		print("top dogs:")
		hasTopDogDied = False
		topDogs = sorted(leaderboard.items(), key=lambda x:x[1], reverse=True)[:3]
		for contestant, score in topDogs:
			print("\t{}: \t\t{}".format(contestant, int(score)))
			if contestant == loser:
				hasTopDogDied = True
		

		# fun
		if hasTopDogDied:
			print("\t\t{}".format('*'.join(["RIP" for i in range(24)])))
			print("\t\t\tLet us remember the legendary \n\t\t\t\t\t\t\t{}, \n\t\t\t\t\t\t\t\t\twho perished from sheer misfortune.".format(loser))
			print("\t\t{}".format('*'.join(["RIP" for i in range(24)])))


		# save stats
		with open(statsPath, 'wb+') as file:
			pickle.dump(stats, file)
