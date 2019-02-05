#!/usr/bin/env python

import sys
import rospy
from timer import Timer
from base import set_speed, get_speed
from std_msgs.msg import UInt8

desired_speed = 0
ticks_count = 0

error_prior = 0

i_time = Timer()

tick_sub = None

def set_desired_speed(s):
	global desired_speed, speed_control
	desired_speed = s
	i_time.start()

def proc(ticks):
	global desired_speed, ticks_count, error_prior
	ticks_count += ticks.data

	if i_time.elapsed_time()<0.1: return

	wheel_circumference = 0.2 #20cm
	rps = ticks_count/10.0/i_time.elapsed_time()
	ticks_count=0

	actual_speed = wheel_circumference * rps #in m/s


	min_speed = 180
	if(desired_speed<=0.1): set_speed(0)
	else:
		error = desired_speed - actual_speed
		der = 2.0*(error_prior-error)/i_time.elapsed_time()
		new_speed = max(min_speed, get_speed() + 50.0*error*i_time.elapsed_time() + der)
		error_prior = error
		set_speed(new_speed)

	#print("Actual speed: " + str(actual_speed))
	i_time.start()


def sc_init():
	global ticks_sub
	print("Speed Controller Active")
	set_speed(0)
	ticks_sub = rospy.Subscriber("/ticks", UInt8, proc, queue_size=None)

def sc_shutdown():
	global ticks_sub
	ticks_sub.unregister()
	set_speed(0)
	print("Speed Controller Inactive")
