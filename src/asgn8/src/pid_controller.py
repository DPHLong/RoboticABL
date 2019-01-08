#!/usr/bin/env python

from timer import Timer


class PID_Controller:
	innerTimer = Timer()
	Kp = 0
	Ki = 0
	Kd = 0
	error_prior = 0
	integral = 0
	bias = 0

	desired_value = 0

	def set_pid_params(self, Kp_, Ki_, Kd_):
		self.innerTimer.start()
		self.Kp, self.Ki, self.Kd = Kp_, Ki_, Kd_

	def update(self, measured_value):
		error = self.desired_value - measured_value
		time = self.innerTimer.elapsed_time()
		self.integral = self.integral + (error * time)
		derivative = (error - self.error_prior)/time
		output = self.Kp*error + self.Ki*self.integral + self.Kd*derivative + self.bias
		self.error_prior = error
		self.innerTimer.start()
		return output
