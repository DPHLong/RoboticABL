#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan

lidar_data = []

def scan_callback(data):
	global lidar_data
	coord_mul = 100 #meter to pixel multiplier

	radii = np.asarray(data.ranges)
	points = []
	ctm_range_min = 0.10 #custom min range to filter out close objects (the car itself)
	ctm_range_max = 1.2 #custom max range to filter out far away objects
	ctm_angle_min = -1.0
	ctm_angle_max = 1.0
	
	for i in range(len(radii)):
		if (radii[i]<data.range_max and radii[i]<ctm_range_max) and (radii[i]>data.range_min and radii[i]>ctm_range_min):
			angle = data.angle_min + data.angle_increment * i
			if not (abs(angle)<2.5):
				x = np.cos(angle) * radii[i] * coord_mul
				y = np.sin(angle) * radii[i] * coord_mul
				points.append([y, -x])
	lidar_data = list(points)
	#print("Scan data updated")

def get_lidar_data():
	global lidar_data
	ret = list(lidar_data)
	lidar_data = []
	return ret

def localCoord_to_worldCoord(points, pos, yaw):
	ret = []
	for p in points:
		#rotate point around origin (car position) by yaw
		p_x = -(p[0]*np.cos(yaw) - p[1]*np.sin(yaw))
		p_y = (p[0]*np.sin(yaw) + p[1]*np.cos(yaw))
		#apply car position
		p_x += pos[0]
		p_y += pos[1]
		#apply localization to lidar distance
		loc_lid_dist = 20
		forward = [math.sin(yaw) * loc_lid_dist, math.cos(yaw) * loc_lid_dist] #car forward vector
		p_x += forward[0]
		p_y += forward[1]
		ret.append([p_x, p_y])
	return ret
