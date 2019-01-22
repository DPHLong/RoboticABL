#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
import cv2
from std_msgs.msg import Int16, UInt8, UInt16
from angle_convert import angle_convert

pub_steering = rospy.Publisher("/steering", UInt8, queue_size=1, latch=True)
pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=1, latch=True)

steering_angle = 0
curret_speed = 0

def set_speed(speed):
	global current_speed
	current_speed = speed
	print("Setting speed:" + str(speed))
	if(speed>450 or speed <0): print("WARN: Invalid Speed Value: " + str(speed))
	else: pub_speed.publish(speed)

def set_steering(steering):
	global steering_angle
	steering_angle = steering
	steering = angle_convert(steering)
	print("Setting steering:" + str(steering))
	pub_steering.publish(steering)

def get_steering():
	global steering_angle
	return steering_angle

def get_speed():
	global current_speed
	return current_speed

def clamp(min_val, max_val, val):
	return min(max_val, max(min_val, val))

def lerp(in_min, in_max, out_min, out_max, val):
	val = clamp(in_min, in_max, val)
	return out_min + (out_max - out_min) * (val - in_min)/(in_max - in_min)

def normalize(vec):
	v = np.array(vec)
	v = v/math.sqrt(sum(v**2))
	return v.tolist()

def angle(v1, v2):
	v1 = normalize(v1)
	v2 = normalize(v2)
	a = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
	if v1[0]*v2[1] - v1[1]*v2[0] < 0: return -a #cross product for negative angles
	return a


