#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np
import math
from timer import Timer
from nav_msgs.msg import Odometry
from trajectory import closest_point, get_distance, closest_point_LookAhead
from trajectory import lane1 as trj
import matplotlib.pyplot as plt
from threading import Lock
from base import angle, set_speed, set_steering, lerp, get_steering

car_position = [0,0]
yaw = 0
mutex = Lock()


total_timer = Timer()

def plot_point(canvas, point, color_='r'):
	canvas.scatter(point[0], point[1], color=color_)

def odom_cb(data):
	global car_position, yaw

	m_to_pix_mul = 100.0
	pos = [data.pose.pose.position.y*m_to_pix_mul, data.pose.pose.position.x*m_to_pix_mul]
	(r, p, y) = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])

	mutex.acquire()
	car_position = pos
	yaw = y
	mutex.release()

	#get car forward vector by converting yaw from polar into cartesian
	forward = [math.sin(y), math.cos(y)]

	#get point to steer towards
	s_pos = closest_point_LookAhead(trj, pos, 50)
	#s_pos = closest_point(trj, pos, return_closest_i = False)

	#get desired forward-vector
	forward_want = [s_pos[0]-pos[0], s_pos[1]-pos[1]]

	#get angle between both vectors
	a = angle(forward, forward_want)

	#set steering
	steering = lerp(-0.5, 0.5, -25, 25, a)
	if(abs(steering-get_steering())>1.0): set_steering(steering)
	

def main(args):
	global car_position, total_error, readings, total_sq_error

	rospy.init_node('odom_follow')
	print("Node started")
	total_timer.start()

	odom_sub = rospy.Subscriber("/localization/odom/4", Odometry, odom_cb, queue_size=10)
	rospy.sleep(1)

	set_speed(320)
	while(total_timer.elapsed_time()<30):
		rospy.sleep(0.1)

	set_speed(0)

	odom_sub.unregister()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
