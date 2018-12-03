#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from ransac import ransac, remove_inliers, line_point_distance, set_params, get_distance
from std_msgs.msg import Int16, UInt8, UInt16

lidar_data = []
prev_m1=0
prev_m2=99999

ax_a, ax_b, ax_c = plt.subplots(1, 3, figsize=(21, 7), facecolor='w')[1]
plot_size = 8.0
plt.show(block=False)

#pub_stop_start = rospy.Publisher("/manual_control/stop_start", Int16, queue_size=100, latch=True)
pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100, latch=True)
pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)

def scan_callback(data):
	global lidar_data
	radii = np.asarray(data.ranges)
	points = []
	
	for i in range(len(radii)):
		if radii[i]<data.range_max and radii[i]>data.range_min:
			angle = data.angle_min + data.angle_increment * i
			x = np.cos(angle) * radii[i]
			y = np.sin(angle) * radii[i]
			points.append([y, -x])
	lidar_data = points
	#print("Scan data updated")

def get_lidar_data():
	global lidar_data
	assert(len(lidar_data)>0)
	return lidar_data

def plot_points(canvas, points):
	canvas.set_xlim([-plot_size,plot_size])
	canvas.set_ylim([-plot_size,plot_size])
	canvas.cla()
	for p in points: canvas.scatter(p[0], p[1], color='r')
	plt.show(block=False)

def plot_line(canvas, m, c):
	canvas.set_xlim([-plot_size,plot_size])
	canvas.set_ylim([-plot_size,plot_size])
	x = np.linspace(-plot_size, plot_size)
	canvas.plot(x, m*x + c)
	plt.show(block=False)

def calibrate():
	#preparations and parameters
	speed = -200
	desired_steering_angle = 112.24
	print("Steering angle: " + str(desired_steering_angle))
	pub_steering.publish(UInt8(desired_steering_angle))

	print("Point 1 reading")
	rospy.sleep(1)
	points1 = get_lidar_data()
	rospy.sleep(1)

	print("Driving started")
	pub_speed.publish(Int16(speed))
	rospy.sleep(1.5)

	print("Point 2 reading")
	points2 = get_lidar_data()
	rospy.sleep(1.5)

	print("Driving stopped")
	#end point
	pub_speed.publish(Int16(0))
	print("Point 3 reading")
	points3 = get_lidar_data()
	
	print("Processing points...")
	plot_points(ax_a, points1)
	plot_points(ax_b, points2)
	plot_points(ax_c, points3)
	x1, y1 = process_lidar_data(ax_a, points1)
	x2, y2 = process_lidar_data(ax_b, points2)
	x3, y3 = process_lidar_data(ax_c, points3)
	print("Car position at point 1: " + str(x1) + " : " + str(y1))
	print("Car position at point 1: " + str(x2) + " : " + str(y2))
	print("Car position at point 1: " + str(x3) + " : " + str(y3))

	#B
	#Slopes of interconnecting lines: http://paulbourke.net/geometry/circlesphere/
	m_a = (y2 - y1)/(x2 - x1)
	m_b = (y3 - y2)/(x3 - x2)

	#center coordinates of circle
	c_x = (m_a*m_b*(y1-y3) + m_b*(x1+x2) - m_a*(x2+x3)) / (2*(m_b - m_a))
	c_y = -(1/m_a) * (c_x - ((x1+x2)/2)) + ((y1+y2)/2)
	print("Circle center: " + str(c_x) + " : " + str(c_y))

	#radius os circle
	c_r = get_distance(c_x, c_y, x1, y1)
	print("Circle radius: " + str(c_r))
	
	#C
	#measured_values:
	lidar_front_wheels_dist = 0.06 #6cm
	lidar_back_wheels_dist = 0.2 #20cm

	L = lidar_front_wheels_dist + lidar_back_wheels_dist
	R_rear = math.sqrt( c_r*c_r - lidar_back_wheels_dist * lidar_back_wheels_dist ) #pythagoras

	v_angle = math.atan(L / R_rear)
	print("Virtual front wheel angle: " + str(v_angle) + "rad, " + str(v_angle*180/math.pi) + "deg")
	

#return x, y coordinates of car depending on lidar points
def process_lidar_data(canvas, points):
	global prev_m1, prev_m2
	set_params(0.05, 100)
	m1, c1 = ransac(points)
	points = remove_inliers(m1, c1, points)
	m2, c2 = ransac(points)

	if abs(math.atan((m1-prev_m1)/(1+m1*prev_m1))) > abs(math.atan((m1-prev_m2)/(1+m1*prev_m2))):
		tmp_m, tmp_c = m1, c1
		m1, c1 = m2, c2
		m2, c2 = tmp_m, tmp_c
	prev_m1 = m1
	prev_m2 = m2

	plot_line(canvas, m1, c1)
	plot_line(canvas, m2, c2)
	x = line_point_distance(m1, c1, [0,0])
	y = line_point_distance(m2, c2, [0,0])
	return x, y


		
def main(args):
	print("Node started")
	rospy.init_node('angle_calibration')
	try:
		rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)
	except rospy.ROSInterruptException:
		pass
	calibrate()
	#rospy.spin()
	plt.show()

if __name__ == '__main__':
	main(sys.argv)
