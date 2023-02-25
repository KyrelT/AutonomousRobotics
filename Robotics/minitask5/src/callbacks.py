#!/usr/bin/env python

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from numpy import inf, array
import random
import math
import tf
import numpy as np
import rospy
import cv2, cv_bridge


class OdomLaserScan:
	def __init__(self):
		# Initialise the subscribers
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
		self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
		self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.grid_callback)
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image,self.image_callback)
		self.explore_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
		self.msg = Twist()
		self.pose = Pose2D()

		self.bridge = cv_bridge.CvBridge()
		self.greenbeaconFound = False
		self.redHydrantfound = False
		self.blueTilesFound = False
		self.greenStarted = False
		self.redStarted = False	
		self.moveToBeacon = False
		self.leftTurn = False
		self.rightTurn = False
		self.beaconArrived = False
		self.announceCount = False
		self.obstacle_found = False
		self.blueTilesMove = False
		self.middle = True
		
		self.action = 1
		self.first_obstacle_turn = True
		self.first_obstacle_move = True
		self.move_around_done = True

		self.distances = np.array([])
    		self.angles = np.array([])


	# Callbacks for occupancy grid subscriber
	def grid_callback(self, msg):
		
        	#rospy.loginfo(msg.data)
		pass


	# Callbacks for laser scan subscriber
	def laser_callback(self, msgScan):
		# Get the distances, angles and the maximum range of each laser, 360 degree.
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
		if self.blueTilesMove is False:
			if (self.greenStarted is True or self.redStarted is True) and self.moveToBeacon is True:
				self.obstacles_avoid(regions)
		else:
			self.blue_tiles_avoid(regions)
			
	
	# Callbacks for Odometry subscriber
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


	def image_callback(self,msg):

		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding = 'bgr8')

		# Original Image
		(h,w) = image.shape[:2]
		image_resized = cv2.resize(image, (w/4,h/4))

		#Masking image
		hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)

		#define green in HSV
		low_green = np.array([48,165,10])
		high_green = np.array([80, 255, 255])
		self.green_mask = cv2.inRange(hsv, low_green, high_green)

		#define blue in HSV
		low_blue = np.array([110,160,150])
		high_blue = np.array([130, 255, 255])
		self.blue_mask = cv2.inRange(hsv, low_blue, high_blue)

		#define red in HSV
		low_red = np.array([0,255,10])
		high_red = np.array([5, 255, 150])
		self.red_mask = cv2.inRange(hsv, low_red, high_red)

		self.h, self.w, d = image_resized.shape

		# only focus on the bottom middle screen for blue mask
		self.blue_mask[0:self.h - self.h/14, :] = 0
		self.blue_mask[:, 0:self.w/2 - 75] = 0
		self.blue_mask[:, self.w/2 + 75:self.w] = 0

		self.green_contours = []
		self.red_contours = []
		self.blue_contours = []

		green_ret, green_thresh = cv2.threshold(self.green_mask, 50, 255, 0)
		(_, self.green_contours, _)= cv2.findContours(green_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		red_ret, red_thresh = cv2.threshold(self.red_mask, 50, 255, 0)
		(_, self.red_contours, _)= cv2.findContours(red_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		blue_ret, blue_thresh = cv2.threshold(self.blue_mask, 50, 255, 0)
		(_, self.blue_contours, _)= cv2.findContours(blue_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
		def get_contour_areas(contours):
			all_areas = []
			for cnt in self.contours:
				area= cv2.contourArea(cnt)
				all_areas.append(area)
			return all_areas

		if self.blueTilesMove is False and self.obstacle_found is False:
			if len(self.green_contours) != 0: # if green object detected
				self.greenbeaconFound = True
			else:
				self.greenbeaconFound = False
				self.greenStarted = False
				if self.redStarted is False:
					self.moveToBeacon = False

			if len(self.red_contours) != 0: # if red object detected
				self.redHydrantFound = True
			else:
				self.redHydrantFound = False
				self.redStarted = False
				if self.greenStarted is False:
					self.moveToBeacon = False

			if len(self.blue_contours) != 0: # if blue object detected
				self.blueTilesFound = True
			else:
				self.blueTilesFound = False	

		# If found blue tiles
		if self.blueTilesFound is True and self.blueTilesMove is False:
			self.blue = cv2.bitwise_and(image_resized, image_resized, mask=self.blue_mask)

			self.blueTiles()

			cv2.imshow("Masked", self.blue) # masked colours
		else:
			if self.greenbeaconFound is True and self.redStarted is False and self.obstacle_found is False and self.blueTilesMove is False:
				self.green = cv2.bitwise_and(image_resized, image_resized, mask=self.green_mask)

				self.move_to_largest_beacon(self.green_mask, self.green_contours, self.green)

				cv2.imshow("Masked", self.green) # masked colours
				self.greenStarted = True

			elif self.redHydrantFound is True and self.greenStarted is False and self.obstacle_found is False and self.blueTilesMove is False:
				self.red = cv2.bitwise_and(image_resized, image_resized, mask=self.red_mask)

				self.move_to_largest_beacon(self.red_mask, self.red_contours, self.red)

				cv2.imshow("Masked", self.red) # masked colours
				self.redStarted = True
				
		# Showing real-time footage
		cv2.imshow("Original", image_resized) # unfiltered vision

		cv2.waitKey(2)

	def blueTiles(self):
		M2 = cv2.moments(self.blue_mask)
		if M2['m00'] > 0:
			#Centroid formula
			cx = int(M2['m10']/M2['m00'])
			cy = int(M2['m01']/M2['m00'])
			cv2.circle(self.blue, (cx, cy), 10, (0,255,0), -1)

			self.blueTilesMove = True
			print("Blue Tiles Found")


	def move_to_largest_beacon(self, mask, contour, robot_vision):
		
		if self.moveToBeacon is False:
			largest_contour= max(contour, key = cv2.contourArea)		

			min_height = int(min(largest_contour[:,:,1]))
			max_height = int(max(largest_contour[:,:,1]))
			min_width = int(min(largest_contour[:,:,0]))
			max_width = int(max(largest_contour[:,:,0]))
					
			self.largest_object = mask.copy()
			self.largest_object[0:min_height, :] = 0
			self.largest_object[max_height:self.h, :] = 0
			self.largest_object[:, 0:min_width] = 0
			self.largest_object[:, max_width:self.w] = 0

		else:
			self.largest_object = mask.copy()
			self.largest_object[:, 0:self.w/2 - 45] = 0
			self.largest_object[:, self.w/2 + 45:self.w] = 0

		M = cv2.moments(self.largest_object)
		if M['m00'] > 0:
			#Centroid formula
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(robot_vision, (cx, cy), 10, (255,255,255), -1) # dot in the middle of object

			# To adjust beacon to middle of screen
			if cx < self.w/2-10: # if object found at the left side of the screen
				print("Beacon Locate Left")
				self.msg.linear.x = 0
				self.msg.angular.z = 0.05
			elif cx > self.w/2+10: # if object found at the right side of the screen
				print("Beacon Locate Right")
				self.msg.linear.x = 0
				self.msg.angular.z = -0.05
			else:
				print("Moving Towards Beacon")
				self.msg.linear.x = 0.1
				self.msg.angular.z = 0	
				self.moveToBeacon = True

			self.explore_pub.publish(self.msg)

	def blue_tiles_avoid(self, regions):
		if self.leftTurn is False and self.rightTurn is False:
			if regions['leftleft'] < regions['rightright']:
				# Go around by turning 90 degree right
				self.rightTurn = True
			else:
				# Go around by turning 90 degree left
				self.leftTurn = True

		elif self.leftTurn is True:
			if self.action == 1: # Turn 90 left
				self.turn(0.1, 'first')
				self.move_around_done = False
			elif self.action == 2: # Move forward
				self.move_forward_blue(0.6, regions)
			elif self.action == 3: # Turn 90 right
				self.turn(-0.1, 'second')
			elif self.action == 4: # Move forward
				self.move_forward_blue(0.8, regions)
			elif self.action == 5: # Turn 90 right
				self.turn(-0.1, 'third')
			elif self.action == 6: # Move forward
				self.move_forward_blue(0.6, regions)
			elif self.action == 7: # Turn 90 left
				self.turn(0.1, 'last')

			if self.move_around_done is True: # After everything done, reset the booleans
				self.leftTurn = False
				self.blueTilesMove = False
				self.action = 1

		elif self.rightTurn is True:
			if self.action == 1: # Turn 90 right
				self.turn(-0.1, 'first')
				self.move_around_done = False
			elif self.action == 2: # Move forward
				self.move_forward_blue(0.6, regions)
			elif self.action == 3: # Turn 90 left
				self.turn(0.1, 'second')
			elif self.action == 4: # Move forward
				self.move_forward_blue(0.8, regions)
			elif self.action == 5: # Turn 90 left
				self.turn(0.1, 'third')
			elif self.action == 6: # Move forward
				self.move_forward_blue(0.6, regions)
			elif self.action == 7: # Turn 90 right
				self.turn(-0.1, 'last')

			if self.move_around_done is True: # After everything done, reset the booleans
				self.rightTurn = False
				self.blueTilesMove = False
				self.action = 1

	def move_forward_blue(self, distance, regions):
		if self.first_obstacle_move is True:
			self.x_init = self.pose.x
			self.y_init = self.pose.y
			self.a_init = self.pose.theta
			self.first_obstacle_move = False
			print("Move Forward")

		if math.sqrt((self.pose.x - self.x_init)**2 + (self.pose.y - self.y_init)**2) < distance and regions['frontleft'] > 0.2 and regions['frontright'] > 0.2:
			self.msg.angular.z = 0
			self.msg.linear.x = 0.1
			self.explore_pub.publish(self.msg)
		else:
			self.first_obstacle_move = True
			self.msg.angular.z = 0
			self.msg.linear.x = 0
			self.explore_pub.publish(self.msg)
			self.action += 1 # After finish, go to the next action

	def obstacles_avoid(self, regions):
		if self.leftTurn is False and self.rightTurn is False and self.beaconArrived is False:
			state_description = ''

			if regions['frontleft'] < 0.4 and regions['frontright'] > 0.4: #found obstacle at front left
				if 0 in self.largest_object[self.h/2 - 1:self.h/2 + 1, self.w/2 - 40:self.w/2 - 38]:
					state_description = 'normal obstacles detected (front left)'
					# Go around by turning 90 degree right
					self.rightTurn = True

				else: 
					state_description = 'beacon obstacle found (front left)'
					# Turn 180 degree
					self.beaconArrived = True
					self.announceCount = True

				self.obstacle_found = True
				self.msg.linear.x = 0
				self.msg.angular.z = 0

				print(state_description)
				self.explore_pub.publish(self.msg)

			elif regions['frontleft'] > 0.4 and regions['frontright'] < 0.4: #found obstacle at front right
				if 0 in self.largest_object[self.h/2 - 1:self.h/2 + 1, self.w/2 + 38:self.w/2 + 40]:
					state_description = 'normal obstacles detected (front right)'
					# Go around by turning 90 degree left
					self.leftTurn = True
				else: 
					state_description = 'beacon obstacle found (front right)'
					# Turn 180 degree
					self.beaconArrived = True
					self.announceCount = True

				self.obstacle_found = True
				self.msg.linear.x = 0
				self.msg.angular.z = 0

				print(state_description)
				self.explore_pub.publish(self.msg)

			elif regions['frontleft'] < 0.4 and regions['frontright'] < 0.4: #found obstacle at front
				if 0 in self.largest_object[self.h/2 - 1:self.h/2 + 1, self.w/2 - 1:self.w/2 + 1]:
					state_description = 'normal obstacles detected (front)'
					if regions['leftleft'] < regions['rightright']:
						# Go around by turning 90 degree right
						self.rightTurn = True
					else:
						# Go around by turning 90 degree left
						self.leftTurn = True

				else: 
					state_description = 'beacon obstacle found (front)'
					# Turn 180 degree
					self.beaconArrived = True
					self.announceCount = True

				self.obstacle_found = True
				self.msg.linear.x = 0
				self.msg.angular.z = 0

				print(state_description)
				self.explore_pub.publish(self.msg)
			else:
				self.obstacle_found = False

		
		elif self.beaconArrived is True:
			if self.action == 1: # Turn 90
				self.turn(0.1, 'first')
				self.move_around_done = False
			elif self.action == 2: # Turn 90
				self.turn(0.1, 'last')

			if self.move_around_done is True: # After everything done, reset the booleans
				self.beaconArrived = False
				self.greenStarted = False
				self.redStarted = False	
				self.moveToBeacon = False
				self.obstacle_found = False
				self.middle = True
				self.action = 1


		elif self.leftTurn is True:
			if self.action == 1: # Turn 90 left
				self.turn(0.1, 'first')
				self.move_around_done = False
			elif self.action == 2: # Move forward
				self.move_forward('first', 'left', regions)
			elif self.action == 3: # Turn 90 right
				self.turn(-0.1, 'second')
			elif self.action == 4: # Move forward
				self.move_forward('second', 'left', regions)
			elif self.action == 5: # Turn 90 right
				self.turn(-0.1, 'third')
			elif self.action == 6: # Move forward
				self.move_forward('last', 'left', regions)
			elif self.action == 7: # Turn 90 left
				self.turn(0.1, 'last')

			if self.move_around_done is True: # After everything done, reset the booleans
				self.leftTurn = False
				self.obstacle_found = False
				self.action = 1

		elif self.rightTurn is True:
			if self.action == 1: # Turn 90 right
				self.turn(-0.1, 'first')
				self.move_around_done = False
			elif self.action == 2: # Move forward
				self.move_forward('first', 'right', regions)
			elif self.action == 3: # Turn 90 left
				self.turn(0.1, 'second')
			elif self.action == 4: # Move forward
				self.move_forward('second', 'right', regions)
			elif self.action == 5: # Turn 90 left
				self.turn(0.1, 'third')
			elif self.action == 6: # Move forward
				self.move_forward('last', 'right', regions)
			elif self.action == 7: # Turn 90 right
				self.turn(-0.1, 'last')

			if self.move_around_done is True: # After everything done, reset the booleans
				self.rightTurn = False
				self.obstacle_found = False
				self.action = 1

	def turn(self, turn_speed, action):
		if self.first_obstacle_turn is True:
			self.x_init = self.pose.x
			self.y_init = self.pose.y
			self.a_init = self.pose.theta
			self.first_obstacle_turn = False
			print("Rotate 90 degree")

		if (max(self.pose.theta - self.a_init, self.a_init - self.pose.theta) > (math.pi/2)):
			self.msg.angular.z = 0
			self.msg.linear.x = 0
			self.explore_pub.publish(self.msg)
			self.first_obstacle_turn = True
			self.action += 1
			if action == 'last':
				self.move_around_done = True
			print("Rotate 90 degree, DONE")

		else:
			self.msg.angular.z = turn_speed
			self.msg.linear.x = 0
			self.explore_pub.publish(self.msg)
			

	def move_forward(self, choice, direction, regions): 
		if self.first_obstacle_move is True:
			self.x_init = self.pose.x
			self.y_init = self.pose.y
			self.a_init = self.pose.theta
			self.first_obstacle_move = False
			print("Move Forward")

		if choice == 'first':
			if direction == 'right':
				if (regions['leftleft'] < 0.4 or regions['leftright'] < 0.4) and regions['frontleft'] > 0.2 and regions['frontright'] > 0.2: 
					self.msg.angular.z = 0
					self.msg.linear.x = 0.1
					self.explore_pub.publish(self.msg)
				else:
					self.first_obstacle_move = True
					self.msg.angular.z = 0
					self.msg.linear.x = 0
					self.explore_pub.publish(self.msg)
					self.action += 1
					self.total_distance = math.sqrt((self.pose.x - self.x_init)**2 + (self.pose.y - self.y_init)**2)

			elif direction == 'left':
				if (regions['rightleft'] < 0.4 or regions['rightright'] < 0.4) and regions['frontleft'] > 0.2 and regions['frontright'] > 0.2:
					self.msg.angular.z = 0
					self.msg.linear.x = 0.1
					self.explore_pub.publish(self.msg)
				else:
					self.first_obstacle_move = True
					self.msg.angular.z = 0
					self.msg.linear.x = 0
					self.explore_pub.publish(self.msg)
					self.action += 1
					self.total_distance = math.sqrt((self.pose.x - self.x_init)**2 + (self.pose.y - self.y_init)**2)

		elif choice == 'second':
			if direction == 'right':
				if (regions['leftleft'] < 0.6 or regions['leftright'] < 0.6) and regions['frontleft'] > 0.2 and regions['frontright'] > 0.2:
					self.msg.angular.z = 0
					self.msg.linear.x = 0.1
					self.explore_pub.publish(self.msg)
				else:
					self.first_obstacle_move = True
					self.msg.angular.z = 0
					self.msg.linear.x = 0
					self.explore_pub.publish(self.msg)
					self.action += 1

			elif direction == 'left':
				if (regions['rightleft'] < 0.6 or regions['rightright'] < 0.6) and regions['frontleft'] > 0.2 and regions['frontright'] > 0.2:
					self.msg.angular.z = 0
					self.msg.linear.x = 0.1
					self.explore_pub.publish(self.msg)
				else:
					self.first_obstacle_move = True
					self.msg.angular.z = 0
					self.msg.linear.x = 0
					self.explore_pub.publish(self.msg)
					self.action += 1

		elif choice is 'last':
			if math.sqrt((self.pose.x - self.x_init)**2 + (self.pose.y - self.y_init)**2) < self.total_distance and regions['frontleft'] > 0.2 and regions['frontright'] > 0.2:
				self.msg.angular.z = 0
				self.msg.linear.x = 0.1
				self.explore_pub.publish(self.msg)
			else:
				self.first_obstacle_move = True
				self.msg.angular.z = 0
				self.msg.linear.x = 0
				self.explore_pub.publish(self.msg)
				self.action += 1



