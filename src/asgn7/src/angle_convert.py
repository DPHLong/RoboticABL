#!/usr/bin/env python

import math
import numpy as np

x_vals = [ -25.0, -13.0, -6.0, 0.0, 6.0, 13.0, 25.0 ]
y_vals = [ 0, 30, 60, 90, 120, 150, 180 ]

def angle_convert(v_angle):
	return np.interp(v_angle, x_vals, y_vals)

