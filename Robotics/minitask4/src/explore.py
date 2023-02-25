#!/usr/bin/env python2

import rospy
import math
import random
import numpy as np
from numpy import inf, array
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import tf
import callbacks

class Explore:
	def __init__(self):
		self.explore_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
		#self.avoid_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
		self.callback = callbacks.OdomLaserScan()
		self.msg = Twist()
		self.obstacle_found = False
    		self.first_run = True
		self.robotrotating = False
		self.x_init = 0
		self.y_init = 0
		self.a_init = 0
		self.travel = 0

	def random_explore(self):

		if self.first_run is True or self.obstacle_found is True: # init everything once it stops
			self.x_init = self.callback.pose.x
			self.y_init = self.callback.pose.y
			self.a_init = self.callback.pose.theta
			self.first_run = False

		#print(self.first_run)

		if math.sqrt((self.callback.pose.x - self.x_init)**2 + (self.callback.pose.y - self.y_init)**2) >= 3:
			random_angle = math.pi * random.random()
			self.robotrotating = True
			print("3m limit, randomise direction")
			print("rotating angle from "+ str(self.a_init) + " -> " + str(random_angle) )
			if random_angle < math.pi:
				while (max(self.callback.pose.theta - self.a_init, self.a_init - self.callback.pose.theta) < random_angle):
					if (max(self.callback.pose.theta - self.a_init, self.a_init - self.callback.pose.theta) > random_angle):
						self.msg.angular.z = 0
						self.pub.publish(self.msg)
					else:
						self.msg.angular.z = 0.3
						self.msg.linear.x = 0
					self.explore_pub.publish(self.msg)
				self.first_run = True
				self.robotrotating = False


	def obstacles_avoid(self, regions):
		linear_x = 0
		angular_z = 0
		state_description = ''

		if regions['frontleft'] < 0.7 and regions['frontright'] > 0.7:
			state_description = 'obstacles detected (front left)'
			linear_x = 0
			angular_z = -0.3
			self.obstacle_found = True
		elif regions['frontleft'] > 0.7 and regions['frontright'] < 0.7:
			state_description = 'obstacles detected (front right)'
			linear_x = 0
			angular_z = 0.3
			self.obstacle_found = True
		elif regions['frontleft'] < 0.7 and regions['frontright'] < 0.7:
			state_description = 'obstacles detected (front)'
			linear_x = 0
			angular_z = -0.3
			self.obstacle_found = True
		else:
			state_description = 'exploring'
			linear_x = 0.3
			angular_z = 0
			self.random_explore()
			self.obstacle_found = False

		#print(state_description)
		self.msg.linear.x = linear_x
		self.msg.angular.z = angular_z
		self.explore_pub.publish(self.msg)
	



