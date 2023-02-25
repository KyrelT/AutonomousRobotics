#!/usr/bin/env python2

import rospy
import math
import random
import numpy as np
import cv2, cv_bridge
import matplotlib.pyplot as plt
import tf
import Main
from numpy import inf, array
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

class Behave:

	def __init__(self):
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.pose = Pose2D()
    		self.obstacle_found = False
    		self.first_run = True
    		self.msg = Twist()
		self.bridge = cv_bridge.CvBridge()
		self.trajectory = list()
		self.green_beacon_found = False
		self.green_found = False
		self.obstacle_leftside_found = False
		self.obstacle_rightside_found = False
		
	
	def callback(self, msg):

    		msg.ranges = array(msg.ranges)
    		msg.ranges[msg.ranges == 0] = inf
    		msg.ranges = tuple(msg.ranges)

    		# region is define from 0 to 360, like a circle
    		regions = {
        		'frontleft': min(min(msg.ranges[0:44]), 10),
        		'leftright': min(min(msg.ranges[45:89]), 10),
        		'leftleft': min(min(msg.ranges[90:134]), 10),
        		'backright': min(min(msg.ranges[135:179]), 10),
        		'backleft': min(min(msg.ranges[180:224]), 10),
        		'rightright': min(min(msg.ranges[225:269]), 10),
        		'rightleft': min(min(msg.ranges[270:314]), 10),
        		'frontright': min(min(msg.ranges[315:359]), 10),
    			}

    		#run the obstacle avoid (regions)
		self.obstacles_avoid(regions)

	def odom_callback(self, msg):
	    # get pose = (x, y, theta) from odometry topic
	    quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
		           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
	    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
	    self.pose.theta = yaw
	    self.pose.x = msg.pose.pose.position.x
	    self.pose.y = msg.pose.pose.position.y
	
	def obstacles_avoid(self, regions):
		linear_x = 0
		angular_z = 0
		state_description = ''
		asdasdasd = ''

		if regions['frontleft'] < 0.5 and regions['frontright'] > 0.5:
			if 0 in self.largest_object[self.h/2 - 1:self.h/2 + 1, self.w/4 - 1:self.w/4 + 1]:
				state_description = 'normal obstacles detected (front left)'
			else: 
				state_description = 'green beacon obstacle found front left'
				self.green_beacon_found = False
				self.green_found = False
			# check if there's obstacles on the side,
			# if there is, boolean set to true to stop adjusting the red dot
			if regions['leftright'] < 0.5 or regions['leftleft'] < 0.5:
				self.obstacle_leftside_found = True
			linear_x = 0
			angular_z = -0.3
			self.obstacle_found = True

		elif regions['frontleft'] > 0.5 and regions['frontright'] < 0.5:
			if 0 in self.largest_object[self.h/2 - 1:self.h/2 + 1, self.w - self.w/4 - 1:self.w - self.w/4 + 1]:
				state_description = 'normal obstacles detected (front right)'
			else: 
				state_description = 'green beacon obstacle found front right'
				self.green_beacon_found = False
				self.green_found = False
			# check if there's obstacles on the side,
			# if there is, boolean set to true to stop adjusting the red dot
			if regions['rightleft'] < 0.5 or regions['rightright'] < 0.5:
				self.obstacle_rightside_found = True
			linear_x = 0
			angular_z = 0.3
			self.obstacle_found = True

		elif regions['frontleft'] < 0.5 and regions['frontright'] < 0.5:
			if 0 in self.largest_object[self.h/2 - 1:self.h/2 + 1, self.w/2 - 1:self.w/2 + 1]:
				state_description = 'normal obstacles detected (front)'
			else:
				state_description = 'green beacon obstacle found'
				self.green_beacon_found = False
				self.green_found = False
			linear_x = 0
			angular_z = 0.3
			self.obstacle_found = True
		else:
			if self.green_beacon_found is False and self.green_found is False:
				state_description = 'random walking'
				linear_x = 0.3
				angular_z = 0
				self.random_walk()
			else:
				state_description = 'Moving towards green beacon'
				linear_x = 0.2
				self.obstacle_rightside_found = False
				self.obstacle_leftside_found = False

			self.obstacle_found = False

		print(state_description)
		self.msg.linear.x = linear_x
		self.msg.angular.z = angular_z
		self.pub.publish(self.msg)

	def image_callback(self, msg):
		
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

		#Original Image		
		(h, w) = image.shape[:2]
		image_resized = cv2.resize(image, (w/4,h/4))

		#Mask image to find green beacon
		hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)   
		low_green = np.array([55, 52, 72])
		high_green = np.array([89, 255, 255])
		self.green_mask = cv2.inRange(hsv, low_green, high_green)
		self.green = cv2.bitwise_and(image_resized, image_resized, mask=self.green_mask)
		self.h, self.w, d = image_resized.shape

		self.contours = []

		ret, thresh = cv2.threshold(self.green_mask, 50, 255, 0)
		(_, self.contours, _)= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		def get_contour_areas(contours):
		    all_areas= []
		    for cnt in self.contours:
			area= cv2.contourArea(cnt)
			all_areas.append(area)
		    return all_areas

		if len(self.contours) is not 0:

			self.green_beacon_moving()
			self.green_found = True
		else:
			self.green_found = False
			self.green_beacon_found = False		

		
		#Show Original image and masked image
		cv2.imshow("original", image_resized)
		cv2.imshow("masked", self.green)
		cv2.imshow("robot_see", self.largest_object)
	
		cv2.waitKey(3)
	
	def green_beacon_moving(self):
		if self.green_beacon_found is False:
			largest_contour= max(self.contours, key = cv2.contourArea)		

			min_height = int(min(largest_contour[:,:,1]))
			max_height = int(max(largest_contour[:,:,1]))
			min_width = int(min(largest_contour[:,:,0]))
			max_width = int(max(largest_contour[:,:,0]))
					
			self.largest_object = self.green_mask
			self.largest_object[0:min_height, :] = 0
			self.largest_object[max_height:self.h, :] = 0
			self.largest_object[:, 0:min_width] = 0
			self.largest_object[:, max_width:self.w] = 0
			self.green_beacon_found = True	
		else:
			self.largest_object = self.green_mask
			self.largest_object[:, 0:self.w/2 - 40] = 0
			self.largest_object[:, self.w/2 + 40:self.w] = 0
	


		M = cv2.moments(self.largest_object)
		if M['m00'] > 0:
			#Centroid formula
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(self.green, (cx, cy), 10, (0,0,255), -1)

			
		if self.obstacle_found is False and self.obstacle_leftside_found is False and self.obstacle_rightside_found is False: 
			
			# To adjust red dot to middle
			if 0 in self.green[:, 0:self.w/2 - 20, 0:1] and 255 in self.green[:, 0:self.w/2 - 20, 2] and (self.obstacle_leftside_found is False or self.obstacle_rightside_found is False):
				print("Beacon In Left")
				self.msg.linear.x = 0
				self.msg.angular.z = 0.05
			elif 0 in self.green[:, self.w/2 +20:self.w, 0:1] and 255 in self.green[:, self.w/2 +20:self.w, 2] and (self.obstacle_leftside_found is False or self.obstacle_rightside_found is False):
				print("Beacon In Right")
				self.msg.linear.x = 0
				self.msg.angular.z = -0.05
			else:
				self.msg.linear.x = 0.1
				self.msg.angular.z = 0			
			self.pub.publish(self.msg)
		

	def random_walk(self):
		global x_init
		global y_init
		global a_init
		self.obstacle_rightside_found = False
		self.obstacle_leftside_found = False

		if self.first_run is True or self.obstacle_found is True or self.green_beacon_found is True: # init everything once it stops
			x_init = self.pose.x
			y_init = self.pose.y
			a_init = self.pose.theta
			self.first_run = False

		if self.green_beacon_found is False:
			if math.sqrt((self.pose.x - x_init)**2 + (self.pose.y - y_init)**2) >= 3:
				random_angle = math.pi * 2 * random.random()
				print("3m limit, randomise direction, angle: " + str(random_angle))
				if random_angle < math.pi:
					while (max(self.pose.theta - a_init, a_init - self.pose.theta) < random_angle):
						if (max(self.pose.theta - a_init, a_init - self.pose.theta) > random_angle):
							self.msg.angular.z = 0
							self.pub.publish(self.msg)
						else:
							self.msg.angular.z = 0.3
							self.msg.linear.x = 0
						self.pub.publish(self.msg)
						print("rotating angle from " + str(a_init) + " -> " + str(random_angle) )
					self.first_run = True
				else:
					while(max(self.pose.theta - a_init, a_init - self.pose.theta) < random_angle/2):
						if(max(self.pose.theta - a_init, a_init - self.pose.theta) > random_angle/2):
					    		self.msg.angular.z = 0
					    		self.pub.publish(self.msg)
						else:
					    		self.msg.angular.z = 0.3
					    		self.msg.linear.x = 0
						self.pub.publish(self.msg)
						print("rotating angle from "+ str(a_init) + " -> " + str(random_angle))
					a_init = self.pose.theta
					while (max(self.pose.theta - a_init, a_init - self.pose.theta) < random_angle/2):
						if(max(self.pose.theta - a_init, a_init - self.pose.theta) > random_angle/2):
					    		self.msg.angular.z = 0
					    		self.pub.publish(self.msg)
						else:
							self.msg.angular.z = 0.3
							self.msg.linear.x = 0
						self.pub.publish(self.msg)
						print("rotating angle from "+ str(a_init) + " -> " + str(random_angle) )
					self.first_run = True



