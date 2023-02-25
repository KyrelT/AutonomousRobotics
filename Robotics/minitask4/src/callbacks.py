#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from numpy import inf, array
import random
import math
import tf
import numpy as np
import rospy


class OdomLaserScan:
	def __init__(self):
		
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
		self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
		self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.grid_callback)
		self.explore_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
		self.msg = Twist()
		self.obstacle_found = False
    		self.first_run = True

		self.pose = Pose2D()
		self.distances = np.array([])
    		self.angles = np.array([])

		self.x_init = 0
		self.y_init = 0
		self.a_init = 0
		self.turn = False
		self.random_angle = 0
	
	def grid_callback(self, msg):
		
        	#rospy.loginfo(msg.data)
		pass


	
	def laser_callback(self, msgScan):
		
		self.distances = np.array([])
    		self.angles = np.array([])
		self.max = msgScan.range_max

    		for i in range(len(msgScan.ranges)):
        		# angle calculation
        		angle = i * msgScan.angle_increment

        		# distance calculation
        		if ( msgScan.ranges[i] > self.max ):
        		    distance = self.max
        		elif ( msgScan.ranges[i] < msgScan.range_min ):
        		    distance = msgScan.range_min
        		else:
        		    distance = msgScan.ranges[i]


        		self.distances = np.append(self.distances, distance)
        		self.angles = np.append(self.angles, angle)

		msgScan.ranges = array(msgScan.ranges)
    		msgScan.ranges[msgScan.ranges == 0] = inf
    		msgScan.ranges = tuple(msgScan.ranges)

    		# region is define from 0 to 360, like a circle
    		regions = {
        		'frontleft': min(min(msgScan.ranges[0:44]), 10),
        		'leftright': min(min(msgScan.ranges[45:89]), 10),
        		'leftleft': min(min(msgScan.ranges[90:134]), 10),
        		'backright': min(min(msgScan.ranges[135:179]), 10),
        		'backleft': min(min(msgScan.ranges[180:224]), 10),
        		'rightright': min(min(msgScan.ranges[225:269]), 10),
        		'rightleft': min(min(msgScan.ranges[270:314]), 10),
        		'frontright': min(min(msgScan.ranges[315:359]), 10),
    			}

    		#run the obstacle avoid (regions)
		self.obstacles_avoid(regions)
	
	def odom_callback(self, msg):
		# get pose = (x, y, theta) from odometry topic
		quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
				msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
		if yaw < 0:
			yaw = 2 * np.pi + yaw
		self.pose.theta = yaw
		self.pose.x = msg.pose.pose.position.x
		self.pose.y = msg.pose.pose.position.y

	# X and Y distance from robot to laser scan
	def distance_scan_xy(self, distances, angles, x_odom, y_odom, theta_odom):

		distances_x = np.array([])
		distances_y = np.array([])

		for (dist, ang) in zip(distances, angles):
			distances_x = np.append(distances_x, x_odom + dist * np.cos(ang + theta_odom))
			distances_y = np.append(distances_y, y_odom + dist * np.sin(ang + theta_odom))

		return (distances_x, distances_y)


	def random_explore(self):

		if self.first_run is True or self.obstacle_found is True: # init everything once it stops
			self.x_init = self.pose.x
			self.y_init = self.pose.y
			self.a_init = self.pose.theta
			self.random_angle = math.pi * random.random()
			self.first_run = False
			print(self.random_angle)

		if math.sqrt((self.pose.x - self.x_init)**2 + (self.pose.y - self.y_init)**2) >= 3 or self.turn is True:
			self.turn = True
			print("3m limit, randomise direction")

			if (max(self.pose.theta - self.a_init, self.a_init - self.pose.theta) > 1):
				self.msg.angular.z = 0
				self.msg.linear.x = 0
				self.first_run = True
				self.turn = False
				self.explore_pub.publish(self.msg)

			else:
				self.msg.angular.z = 0.2
				self.msg.linear.x = 0
				self.first_run = False
				self.turn = True
				self.explore_pub.publish(self.msg)



	def obstacles_avoid(self, regions):


		if regions['frontleft'] < 0.6 and regions['frontright'] > 0.6 and self.turn is False:

			self.msg.linear.x = 0
			self.msg.angular.z = -0.2
			self.explore_pub.publish(self.msg)
			self.obstacle_found = True

		elif regions['frontleft'] > 0.6 and regions['frontright'] < 0.6 and self.turn is False:

			self.msg.linear.x = 0
			self.msg.angular.z = 0.2
			self.explore_pub.publish(self.msg)
			self.obstacle_found = True

		elif regions['frontleft'] < 0.8 and regions['frontright'] < 0.8 and self.turn is False:

			self.msg.linear.x = 0
			self.msg.angular.z = 0.2
			self.explore_pub.publish(self.msg)
			self.obstacle_found = True
		else:

			self.msg.linear.x = 0.2
			self.msg.angular.z = 0
			self.explore_pub.publish(self.msg)

			self.random_explore()
			self.obstacle_found = False




