#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
from timer import Timer
from sensor_msgs.msg import Image
from base import to_white_points, get_steering, set_steering, set_speed
from ransac import set_params, ransac

count = -1

def controller(x_want, x_is, Kp):
	return Kp * (x_want - x_is)

def run(data):
	t = Timer()

	global count
	count += 1
	if count % 10 != 0: return

	t.start()
	wp = to_white_points(data)
	assert(len(wp)>0)

	m, c = ransac(wp)

	center = 120
	offset = (center - c) / m
	print(offset)

	angle_set = -controller(center, offset, 0.167)
	if(abs(angle_set-get_steering())>1.0): set_steering(angle_set)
	#print("offset")
	#print("total:" + str(t.elapsed_time()))


def main(args):
	rospy.init_node('steering_test')
	print("Node started")
	set_params(3.0, 50)
	set_speed(0)
	set_steering(0)
	image_sub = rospy.Subscriber("/camera/color/image_raw",Image, run, queue_size=1)

	rospy.sleep(1)
	set_speed(180)
	rospy.sleep(20)
	set_speed(0)

	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
