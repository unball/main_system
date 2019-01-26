import numpy as np

def step(x, bias):
	x[x>=bias] = 1
	x[x<bias] = 0
	return x

def sigmoid(x,bias):# differentiable alternative to the step
	return 1 / (1 + np.exp(-(x-bias))) 

def ant_sigmoid(x,bias):
	return -1 / (1 + np.exp(-(x-bias))) + 1 

def gaussian(x, mu, sigma):	#for simetric megafunctions (e.g. orientation), max = 1 (not standardized)
	return np.exp(-(x-mu)**2 / (2*sigma))

def relu(x, bias, m):# linear function (for x above bias and below 1), m is the slop (positive)
	return np.minimum(np.maximum(0, m*(x-bias)), 1)

def ant_relu(x,bias,m):# m is negative
	return np.maximum(np.minimum(m*(x-bias)+1,1),0)
 	
def parallelogram(x, bias, bias2, end): #bias < bias2
	m1 = 1. /(bias-begin)
	m2 = -1. /(end-bias2)
	rising = relu(x, begin, m1)
	grow = ant_relu(x,bias2,m2)
	res = np.array([ np.insert(rising,0,1), np.insert(grow,len(grow),1) ])
	return res.min(0)


#recive 1 scalar(x), and arrays, return an array of relevance
def fuzzy(x,begin,middle, end):
	m1 = 1. /(middle[1:]-begin)
	m2 = -1. /(end-middle[:-1])
	rising = relu(x, begin, m1)
	grow = ant_relu(x,middle[:-1],m2)
	res = np.array([ np.insert(rising,0,1), np.insert(grow,len(grow),1) ])
	return res.min(0)
	
"""
def fuzzy(posy, begins, middles, ends):
	fu = triangle(pos,begins, middles, ends)
	defu = sum(fu * np.array([-2,-1,0,1,2]))
	return max(triangle(defu,begins, middle, ends)) #estado
"""
#recive 1 or 2, arrays of relevance (out of fuzzy), and a matrix numbers (FAM)
# and return a sum of result
def defuzzy(in1,FAM,*in2):
	out = np.matrix(in1)
	if len(in2)!=0:
		out = np.matrix(out).transpose() * np.matrix(in2)
	out = np.multiply(FAM,out)
	return np.sum(out)

	
def strike():
	pass

def defense():
	pass

def orientation():
	pass

def forward():
	pass

"""main task
orib = [0,45]
orim = [0,45,90]
oriend = [45,90]

disballopb = [0,65,110,175]
disballopm = [0,65,110,175,220]
disballope = [65,110,175,220]

neardisb = [-20,-10,0,10]
neardism = [-20,-10,0,10,20]
neardise = [-10,0,10,20]

outb = [0,2.5,5,7.5]
outm = [0,2.5,5,7.5,10]
oute = [2.5,5,7.5,10]
"""

"""offense strategy
	

"""

""" shot
anglerm = [,0,]

"""



"""
DEFINE ROBOTS 

FAM = [[-1, -.9, -.2, 0, .4],[-.6,-.2,0,.6,.8],[.2,.4,.8,.9,1]]
tops_dist = [35,50,85,120,135]
tops_ori = [0, .5, 1]

ori = dot(norm(v_robot) , norm((pos_ball - pos_robot)))
dist = abs(CG - pos_robot) 

CG - centro do gol

"""