import math
import numpy as np
import random

def initParticles(particleNum, imageHeight, imageWidth):
	particles = np.zeros((particleNum,3), dtype=float)
	for i in range(particleNum):	
		particles[i][0] = random.randint(0,imageWidth)
		particles[i][1] = random.randint(0,imageHeight)
	
	return particles

# Takes in weights and returns 
def resample_particles(curPart, desiredParticleSize):
	weights = curPart[...,2]
	newIndices = np.random.choice(len(curPart),desiredParticleSize, p=weights)
	
	newParticles = np.zeros(len(desiredParticleSize),3, dtype=float)
	curIndex = 0

	for i in newIndices:
		newParticles[0] = curPart[i] 

	return newParticles

def euclidean_distance(curPosit, virtualPosit):
	return math.sqrt(math.square(virtualPosit[0]-curPosit[0])+math.square(virtualPosit[1]-curPosit[1]))

def update(curPart, curPosit):
	for i in range(len(curPart)):
		curPart[i][2] = euclidean_distance(curPosit,curPart[i])
		total = total + curPart[i][2]

	curPart[...,2] = np.divide(curPart[...,2],total)

def findBestCommand(particles):
	return particles[np.unravel_index(particles[...,2].argmax(), particles[...,2].shape)]