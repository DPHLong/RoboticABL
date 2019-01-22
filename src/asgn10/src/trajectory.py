#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt

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
		n = clamp(0.0, 1.0, n)
		q = n
		i = 0
		for i in range(len(self.shape_weights)):
			if q - self.shape_weights[i]<=0: break
			q -= self.shape_weights[i]
		return self.shapes[i], (q / self.shape_weights[i])


	def get_point(self, n):
		s, i = self.get_shape_and_index(n)
		#print(s.get_point(i))
		return s.get_point(i)

def clamp(min_val, max_val, val):
	return min(max_val, max(min_val, val))

def plot_point(canvas, point, color_='r'):
	canvas.scatter(point[0], point[1], color=color_)
	plt.show(block=False)

def get_distance(p1, p2):
  return math.sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]))

def closest_point(trj, point, return_closest_i = False):
	rul = 1000.0 #resulution until linear interpolation
	it = 1.0 / rul

	#get closest point
	min_dist = 999999999.0
	closest_point = [-1, -1]
	closest_i = 0.0
	i = 0.0
	while(i<=1.0):
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
	# return trj.get_point(i_get)

def get_shape_length(shape):
	it = 1.0 / 1000.0
	i = 0.0
	total_dist = 0.0

	while i+it<=1.0:
		total_dist += get_distance(shape.get_point(i), shape.get_point(i+it))
		i += it
	return total_dist


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

#inner lane
bez1 = Bezier([82.0, 395.0], [82.0, 592.0], [353.0, 592.0], [353.0, 395.0])
line1 = Line([353.0, 395.0], [353.0, 204.0])
bez2 = Bezier([353.0, 204.0], [353.0, 15.0], [90.0, 23.0], [82.0, 204.0])
line2 = Line([82.0, 204.0], [82.0, 395.0])

lane1 = Trajectory([bez1, line1, bez2, line2])

#outer lane
bez1 = Bezier([47.0, 395.0], [47.0, 632.0], [384.0, 632.0], [384.0, 395.0])
line1 = Line([384.0, 395.0], [384.0, 204.0])
bez2 = Bezier([384.0, 204.0], [384.0, -31.0], [47.0, -31.0], [47.0, 204.0])
line2 = Line([47.0, 204.0], [47.0, 395.0])

lane2 = Trajectory([bez1, line1, bez2, line2])

print(closest_point_LookAhead(lane1, [300,200], 0.5))
print(closest_point_LookAhead(lane1, [100,100], 0.2))

"""
ax = plt.subplots(1, 1, figsize=(4.3, 6), dpi=80, facecolor='w')[1]
ax.set_xlim([0,430])
ax.set_ylim([0,600])
#ax.cla()

i = 0
it = 15
while i<1500:
	plot_point(ax, closest_point_LookAhead(trj, [0,0], i))
	i+=it
plt.show()

i = 0
it = 0.01
while(i <= 1.0):
	plot_point(ax, trj.get_point(i))
	plot_point(ax, o_trj.get_point(i))
	#plot_point(ax, s_trj.get_point(i))
	plot_point(ax, lane1.get_point(i))
	plot_point(ax, lane2.get_point(i))
	i = i + it


plt.show()
"""

