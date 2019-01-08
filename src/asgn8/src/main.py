#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
from timer import Timer
from sensor_msgs.msg import Image
from base import to_white_points, get_steering, set_steering, set_speed, lerp, get_speed
from ransac import set_params, ransac
from speed_controller import init, set_desired_value
import matplotlib.pyplot as plt

ax = plt.subplots(1, 1, figsize=(7, 7), facecolor='w')[1]
plot_size = 10.0
plt.show(block=False)
ax.set_xlim([-plot_size,plot_size])
ax.set_ylim([-plot_size,plot_size])
ax.cla()

count = -1

total_timer = Timer()

def plot_point(canvas, point):
	canvas.scatter(point[0], point[1], color='r')
	plt.show(block=False)

def run(data):
	global count, prev_m, prev_c
	count += 1
	if count % 10 != 0: return

	print("loop")

	wp = to_white_points(data)
	assert(len(wp)>0)

	m, c = ransac(wp)

	row = 240 #from bottom up
	ip = (row-c)/m
	print("actual ip " + str(ip))
	if(ip>2000 or ip<-2000): return
	ip = lerp(-300, 600, -1, 1, ip)

	base_angle = ip * 0.7
	print("Expected angle: " + str(base_angle))

	ang = math.atan(1.0/m)
	print("actual ang " + str(ang))
	if(ang>3 or ang<-3): return
	ang = lerp(base_angle-0.6, base_angle+0.6, -1, 1, ang)

	print("IP: " + str(ip) + "  AN: " + str(ang))

	st_angle = 20
	st_offset = 30

	angle_set = -ang * st_angle + ip * st_offset

	if(abs(angle_set-get_steering())>1.0): set_steering(angle_set)

	#if(abs(get_steering())>15.0): set_desired_speed(0.8)
	#else: set_desired_speed(1.5)

	#larger ransac while in curves, smaller while not??


def main(args):
	rospy.init_node('follow')
	print("Node started")
	total_timer.start()
	set_speed(0)
	rospy.sleep(1)
	init() #speed_controller
	set_desired_value(1.2)
	#set_speed(180)
	while(total_timer.elapsed_time() < 10.0): plot_point(ax, [total_timer.elapsed_time(), get_speed()])
	rospy.sleep(10)
	"""
	set_params(6.0, 25)
	set_speed(0)
	set_steering(0)
	image_sub = rospy.Subscriber("/camera/color/image_raw",Image, run, queue_size=None)

	rospy.sleep(1)
	rospy.sleep(40)"""
	set_speed(0)
	set_desired_value(0.0)
	plt.show()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
