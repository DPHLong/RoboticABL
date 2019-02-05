#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt
from base import clamp, get_distance


class Line:
	p1 = np.array([0.0, 0.0])
	p2 = np.array([0.0, 0.0])

	def __init__(self, p1_, p2_):
		self.p1 = np.array(p1_)
		self.p2 = np.array(p2_)

	def get_point(self, n):
		assert(n>=0.0 and n<=1.0)
		return (self.p1 + n*(self.p2-self.p1)).tolist()


class Bezier:
	p1 = np.array([0.0, 0.0])
	p2 = np.array([0.0, 0.0])
	p3 = np.array([0.0, 0.0])
	p4 = np.array([0.0, 0.0])

	def __init__(self, p1_, p2_, p3_, p4_):
		self.p1 = np.array(p1_)
		self.p2 = np.array(p2_)	
		self.p3 = np.array(p3_)	
		self.p4 = np.array(p4_)	

	def get_point(self, n):
		assert(n>=0.0 and n<=1.0)
		point = (1.0-n)**3.0*self.p1 + 3.0*(1.0-n)**2.0*n*self.p2 + 3.0*(1.0-n)*n**2.0*self.p3 + n**3.0*self.p4
		return point.tolist()

class Trajectory:
	shapes = []
	shape_lengths = []
	shape_weights = []
	total_length = 0

	def __init__(self, shapes_):
		self.shapes = shapes_
		self.shape_lengths = [get_shape_length(s) for s in self.shapes]
		self.total_length = sum(self.shape_lengths)
		self.shape_weights = [x / self.total_length for x in self.shape_lengths]

	def get_shape_and_index(self, n):
		n = max(0.0, n % 1.0)
		q = n
		i = 0
		for i in range(len(self.shape_weights)):
			if q - self.shape_weights[i]<=0: break
			q -= self.shape_weights[i]
		return self.shapes[i], (q / self.shape_weights[i])


	def get_point(self, n):
		s, i = self.get_shape_and_index(n)
		return s.get_point(i)

def closest_point(trj, point, return_closest_i = False):
	samples = 15
	s_it = 1.0 / samples

	dsts = [get_distance(trj.get_point(i*s_it), point) for i in range(0, samples)]

	#as shape is convex we know the closest point is somewhere in [c-s_it, c+s_it]
	c = dsts.index(min(dsts)) * s_it

	#previous implementation follows
	rul = 100.0 #resulution until linear interpolation
	it = 1.0 / rul

	#get closest point
	min_dist = 999999999.0
	i = c - s_it
	closest_i = i
	closest_point = [-1, -1]
	while(i<=c + s_it):
		p = trj.get_point(i)
		dist = get_distance(p, point)
		if dist<min_dist:
			min_dist = dist
			closest_point = p
			closest_i = i
		i = i + it

	#get closest neighbouring point
	p_previous = trj.get_point(closest_i - it)
	p_next = trj.get_point(closest_i + it)

	if get_distance(p_previous, point)<get_distance(p_next, point): closest_neigh = p_previous
	else: closest_neigh = p_next

	#get closest point to point on line segment AB
	AB = [closest_neigh[0] - closest_point[0], closest_neigh[1] - closest_point[1]]
	AP = [point[0] - closest_point[0], point[1] - closest_point[1]]
	lengthSqrAB = AB[0]**2.0 + AB[1]**2.0
	if(lengthSqrAB == 0.0): lengthSqrAB = 0.00001
	t = (AP[0]*AB[0] + AP[1]*AB[1]) / lengthSqrAB

	closest_point_final = [closest_point[0]+t*AB[0], closest_point[1]+t*AB[1]]

	if(return_closest_i):
		lin_offset = get_distance(closest_point, closest_point_final) / trj.total_length
		closest_i = closest_i + lin_offset
		return closest_i

	return closest_point_final

def closest_point_LookAhead(trj, point, dist):
	i = closest_point(trj, point, return_closest_i = True)
	i_get = (i + (dist / trj.total_length)) % 1.0
	return trj.get_point(i_get), trj.get_point(i)

def get_shape_length(shape):
	it = 1.0 / 1000.0
	i = 0.0
	total_dist = 0.0

	while i+it<=1.0:
		total_dist += get_distance(shape.get_point(i), shape.get_point(i+it))
		i += it
	return total_dist

"""
#singular line in the middle
s_line = Line([64.0, 208.0], [64.0, 372.0])

s_trj = Trajectory([s_line])


#outer loop
o_bez1 = Bezier([31.0, 395.0], [31.0, 653.0], [400.0, 653.0], [400.0, 395.0])
o_line1 = Line([400.0, 395.0], [400.0, 204.0])
o_bez2 = Bezier([400.0, 204.0], [400.0, -52.0], [31.0, -52.0], [31.0, 204.0])
o_line2 = Line([31.0, 204.0], [31.0, 395.0])

o_trj = Trajectory([o_bez1, o_line1, o_bez2, o_line2])


#inner loop
bez1 = Bezier([95.0, 395.0], [95.0, 568.0], [336.0, 568.0], [337.0, 395.0])
line1 = Line([337.0, 395.0], [337.0, 204.0])
bez2 = Bezier([337.0, 204.0], [336.0, 33.0], [95.0, 33.0], [95.0, 204.0])
line2 = Line([95.0, 204.0], [95.0, 395.0])

trj = Trajectory([bez1, line1, bez2, line2])
"""

#inner lane
bez1 = Bezier([82.0, 395.0], [82.0, 592.0], [353.0, 592.0], [353.0, 395.0])
line1 = Line([353.0, 395.0], [353.0, 204.0])
bez2 = Bezier([353.0, 204.0], [353.0, 15.0], [90.0, 23.0], [82.0, 204.0])
line2 = Line([82.0, 204.0], [82.0, 395.0])

lane1 = Trajectory([bez1, line1, bez2, line2])

#outer lane
bez1 = Bezier([47.0, 395.0], [47.0, 632.0], [384.0, 632.0], [384.0, 395.0])
line1 = Line([384.0, 395.0], [384.0, 204.0])
bez2 = Bezier([384.0, 204.0], [384.0, -31.0], [55.0, -23.0], [47.0, 204.0])
line2 = Line([47.0, 204.0], [47.0, 395.0])

lane2 = Trajectory([bez1, line1, bez2, line2])
