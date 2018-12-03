#!/usr/bin/env python

import math
import numpy as np

x_vals = [ -26.994, -22.846, -15.153, -6.147, 2.145, 13.181, 25.404 ]
y_vals = [ 0, 30, 60, 90, 120, 150, 180 ]

def angle_convert(v_angle):
	return np.interp(v_angle, x_vals, y_vals)

a=-25
while a<=25:
	print("Parameter for virtual angle " + str(a) + ": " + str(angle_convert(a)))
	a += 5
