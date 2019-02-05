#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np
import math
from timer import Timer
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from lidar import get_lidar_data, localCoord_to_worldCoord, scan_callback
from trajectory import closest_point, get_distance, closest_point_LookAhead
from trajectory import lane1 as lane1
from trajectory import lane2 as lane2
import matplotlib.pyplot as plt
import thread
from base import angle, set_speed, set_steering, lerp, get_steering

# =============================

ax = plt.subplots(1, 1, figsize=(4.3, 6), dpi=80, facecolor='w')[1]
ax.set_xlim([430,0])
ax.set_ylim([0,600])

# =============================

car_position = [0,0]
yaw = 0

# =============================

trj = lane1
trj2 = lane2



total_timer = Timer()

def plot_point(canvas, point, color_='r'):
	canvas.scatter(point[0], point[1], color=color_)

def obj_on_lane(lane, objs):
	max_dist_lane = 5
	for o in objs:
		cp = closest_point(lane, o)
		if (get_distance(o, cp) <= max_dist_lane):
			return True, cp
	return False, None

def steering(pos, y):
	#get car forward vector by converting yaw from polar into cartesian
	forward = [math.sin(y), math.cos(y)]

	#get point to steer towards
	s_pos, c_pos = closest_point_LookAhead(trj, pos, 50)
	# s_pos = closest_point(trj, pos, return_closest_i = False)

	#get desired forward-vector
	forward_want = [s_pos[0]-pos[0], s_pos[1]-pos[1]]

	#get angle between both vectors
	a = angle(forward, forward_want)

	#set steering
	steering = lerp(-0.5, 0.5, -25, 25, a)
	if(abs(steering-get_steering())>1.0): set_steering( steering)


def odom_cb(data):
	global car_position, yaw, trj, trj2

	m_to_pix_mul = 100.0
	pos = [data.pose.pose.position.y*m_to_pix_mul, data.pose.pose.position.x*m_to_pix_mul]
	(r, p, y) = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])

	thread.start_new_thread(steering, (pos, y))

	car_position = pos
	yaw = y

	#coordinates of objects
	l_data = get_lidar_data()
	if len(l_data)<=0: return
	obstacles = localCoord_to_worldCoord(l_data, pos, y)

	#determine if object on current lane
	objs_on_current_lane, cp = obj_on_lane(trj, obstacles)

	if not objs_on_current_lane: 
		#print("Current Lane clear")
		return

	objs_on_other_lane, cp2 = obj_on_lane(trj2, obstacles)

	if objs_on_other_lane and get_distance(pos, cp)<100:
		print("Obstacles on both lanes")
		set_speed(0)
		return

	elif not objs_on_other_lane:
		print("Lane swapped")
		trj, trj2 = trj2, trj


def main(args):
	global car_position

	rospy.init_node('obstacle_avoid')
	print("Node started")
	total_timer.start()

	lidar_sub = rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=20)
	rospy.sleep(1)
	odom_sub = rospy.Subscriber("/localization/odom/5", Odometry, odom_cb, queue_size=None)
	rospy.sleep(1)


	set_speed(270)
	while(total_timer.elapsed_time()<60):
		#=============================
		tmp_cp = car_position
		tmp_y = yaw

		obs = localCoord_to_worldCoord(get_lidar_data(), tmp_cp, tmp_y)
		for p in obs: plot_point(ax, p, 'r')

		plot_point(ax, tmp_cp, 'g')

		plt.pause(0.0001)
		plt.show(block=False)
		#============================

	set_speed(0)

	odom_sub.unregister()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
