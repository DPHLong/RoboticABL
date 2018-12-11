#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16, UInt8, UInt16
from angle_convert import angle_convert

bridge = CvBridge()

steering_angle = 0

def to_white_points(data):
	global bridge

	try: image_cv = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e: print(e)

	#lower image resolution
	image_cv = cv2.resize(image_cv, (320, 240), cv2.INTER_AREA)

	min_x = 40
	max_x = 280
	min_y = 60
	max_y = 210
	image_cv = image_cv[min_y:max_y, min_x:max_x]

	#image to grey
	image_grey=cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)

	#Image to binary
	bi_gray_max = 255
	bi_gray_min = 220
	ret, image_bin=cv2.threshold(image_grey, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);
	image_pub_bin.publish(bridge.cv2_to_imgmsg(image_bin, "mono8"))

	nz = cv2.findNonZero(image_bin)
	white_points = nz[:,0]
	return white_points

"""
def draw_line(m, c, max_x):
	global bridge, image_cv
	p1 = (int(c), 0)
	p2 = (int(m*(max_x-1)+c), int(max_x-1))

	image_line = cv2.line(image_cv, p2, p1, (0, 0, 255), 3)
	image_pub_line.publish(bridge.cv2_to_imgmsg(image_line, "bgr8"))
"""

def set_speed(speed):
	print("Setting speed:" + str(speed))
	pub_speed.publish(speed)

def set_steering(steering):
	global steering_angle
	steering_angle = steering
	print("Setting steering:" + str(steering))
	pub_steering.publish(angle_convert(steering))

def get_steering():
	global steering_angle
	return steering_angle


pub_steering = rospy.Publisher("/steering", UInt8, queue_size=1, latch=True)
pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=1, latch=True)
image_pub_bin = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)
image_pub_line = rospy.Publisher("/image_processing/line_img",Image, queue_size=1)
