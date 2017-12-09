import math
import numpy as np
samples =  [0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 0.99]

median = 0.01
base = 2.6
lamb = math.log(2.0, base)/median

#things = [10 ** ((1-sample)*10) for sample in samples]
things = [-math.log(1-u, base)/lamb for u in samples]

# for thing in things:
# 	print("{:.6f}".format(thing**2))
# print("hmm")



def limitRangeToBox(angle, x, y):
	# bound to just top right corner since it is symmetric
	angle = abs(angle)
	thing = angle - math.pi/2
	if thing > 0.0: # if it aims down, reflect it in x-axis
		angle = math.pi/2 - thing
	e = 0.000001
	angle = np.clip(angle, e, math.pi/2 - e)
	rangeLimitedByX = y / math.cos(math.pi/2 - angle)
	rangeLimitedByY = x / math.cos(angle)
	limitedRange = min(rangeLimitedByX, rangeLimitedByY)
	return limitedRange

# samples = [0.0, 0.2, math.pi/4.0, math.pi/2, -0.2]
# print([limitRangeToBox(ang, 1,1) for ang in samples])

samples = np.linspace(10, 200, 20)
baseSpeed = [maxDist + max(250*math.sqrt((max(maxDist, 51)-50)/50.0) - 150, 0) + 60 for maxDist in samples]



normal = [maxDist + max(250*math.sqrt((max(maxDist, 31)-20)/100.0) - 150, 0) + 50 for maxDist in samples]
diffs = [a-b for a,b in zip(baseSpeed, normal)]
for i in range(20):
	print("{:.0f}m: {:.0f}".format(samples[i], diffs[i]))


for i in range(20):
	print("{:.0f}m: {:.0f}".format(samples[i], baseSpeed[i]))