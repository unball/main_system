#!/usr/bin/env python3

import rospy
from math import pi
from math import fabs

number_of_robots = 3

convertion = (250*19) / 100
wheel_reduction = 1/3.0
r = 0.03
L = 0.075

max_tics_per_s = 70000.
encoder_resolution = 512.*19
max_motor_speed = (max_tics_per_s) / encoder_resolution

number_of_robots = 3

class speed:
	def __init__(self):
		self.right = 0
		self.left = 0

	def divideEachSide(self, value):
		self.right = self.right/value
		self.left = self.left/value

	def multiplyEachSide(self,value):
		self.right = self.right*value
		self.left = self.left*value     
	def normalize(self, value):
		if fabs(self.right) >= fabs(self.left):
			self.left = value * self.left/fabs(self.right)
			self.right = value * self.right/fabs(self.right)
		elif fabs(self.left) >= fabs(self.right):
			self.right = value * self.right/fabs(self.left)
			self.left = value * self.left/fabs(self.left)
  


def speeds2motors(v,w):
		wheels = speed()
		wheels.right = (-v + (L/2)*w) / r
		wheels.left = (-v - (L/2)*w) / r
		wheels.divideEachSide(2*pi)
		wheels.multiplyEachSide(wheel_reduction)

		if fabs(wheels.right) > max_motor_speed or fabs(wheels.left) > max_motor_speed:
			wheels.normalize(max_motor_speed)

		wheels.multiplyEachSide(convertion)

		return int(wheels.left), int(wheels.right)
