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

pub = None
wall_found = False
obstacle_found = False
first_run = True

def main():
	global pub
	global pose
	global trajectory

	rospy.init_node('reading_laser')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
	odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
	pose = Pose2D()
	trajectory = list()
	sub = rospy.Subscriber('/scan',LaserScan, callback)
	rospy.spin()

def callback(msg):
	
	msg.ranges = array(msg.ranges)
	msg.ranges[msg.ranges == 0] = inf
	msg.ranges = tuple(msg.ranges)

	#region is define from 0 to 360, like a circle
	regions = {
		'frontleft': min(min(msg.ranges[0:44]),10),
		'leftright': min(min(msg.ranges[45:89]),10),
		'leftleft': min(min(msg.ranges[90:134]),10),
		'backright': min(min(msg.ranges[135:179]),10),
		'backleft': min(min(msg.ranges[180:224]),10),
		'rightright': min(min(msg.ranges[225:269]),10),
		'rightleft': min(min(msg.ranges[270:314]),10),
		'frontright': min(min(msg.ranges[315:359]),10),
		}


	if wall_found is False:
		obstacles_avoid(regions)
	else:
		wall_following(regions)

def obstacles_avoid(regions):
	# next time use functions to define each regions
	# more flexible when more functions are adding up together
	global msg
	global first_run
	global wall_found
	global obstacle_found
	msg = Twist()
	linear_x = 0
	angular_z = 0
	state_description = ''
	if regions['frontleft'] > 0.5 and regions['frontright'] > 0.5:
		state_description = 'no obstacles in front, random walk'
		linear_x = 0.3		
		angular_z = 0
		random_walk()
		obstacle_found = False
	elif regions['frontleft'] < 0.5 and regions['frontright'] > 0.5:
		state_description = 'obstacles found in front left'
		linear_x = 0		
		angular_z = -0.3
		obstacle_found = True
	elif regions['frontleft'] > 0.5 and regions['frontright'] < 0.5:
		if regions['rightleft'] < 0.5 and regions['rightright'] < 0.5:
			state_description = 'wall at right side, wall found'
			linear_x = 0.3		
			angular_z = 0
			wall_found = True
		else:
			state_description = 'obstacles found in front right'
			linear_x = 0		
			angular_z = 0.3
			obstacle_found = True
	elif regions['frontleft'] < 0.5 and regions['frontright'] < 0.5:
		state_description = 'obstacles found in front'
		linear_x = 0		
		angular_z = 0.3
		obstacle_found = True
	else:
		state_description = 'unknown case'

	print(state_description)
	msg.linear.x = linear_x
	msg.angular.z = angular_z
	pub.publish(msg)

def wall_following(regions):
	global msg
	if regions['rightleft'] < 0.5 and regions['rightright'] < 0.5 and (regions['frontright'] < 0.4 or regions['frontleft'] < 0.4):
		state_description = 'wall found - front right obstacle found'
		linear_x = 0	
		angular_z = 0.3
	elif regions['rightleft'] > 0.4 and regions['rightright'] > 0.2 and regions['frontright'] > 0.7:
		state_description = 'wall found - turn toward right wall'
		linear_x = 0.1	
		angular_z = -0.7
	else:
		state_description = 'wall found - follow right wall'
		linear_x = 0.1		
		angular_z = 0
	print(state_description)
	msg.linear.x = linear_x
	msg.angular.z = angular_z
	pub.publish(msg)

def random_walk():
	global msg
	global first_run
	global x_init
	global y_init
	global a_init
	global obstacle_found
	global wall_found
	
	if first_run is True or obstacle_found is True and wall_found is False: # init everything once it stops
		x_init = pose.x
		y_init = pose.y
		a_init = pose.theta
		first_run = False
	
	if math.sqrt((pose.x - x_init)**2 + (pose.y - y_init)**2) >= 3:
		random_angle = math.pi * 2 * random.random()
		print("3m limit, randomise direction, angle: " + str(random_angle))
		if random_angle < math.pi:
		
			while (max(pose.theta - a_init, a_init - pose.theta) < random_angle):
				if (max(pose.theta - a_init, a_init - pose.theta) > random_angle):
					msg.angular.z = 0
					pub.publish(msg)
				else:
					msg.angular.z = 0.3
					msg.linear.x = 0
				pub.publish(msg)
				print("rotate less than half")
			first_run = True
		else:
			while (max(pose.theta - a_init, a_init - pose.theta) < random_angle/2):
				if (max(pose.theta - a_init, a_init - pose.theta) > random_angle/2):
					msg.angular.z = 0
					pub.publish(msg)
				else:
					msg.angular.z = 0.3
					msg.linear.x = 0
				pub.publish(msg)
				print("rotate more than half, first half")
			a_init = pose.theta
			while (max(pose.theta - a_init, a_init - pose.theta) < random_angle/2):
				if (max(pose.theta - a_init, a_init - pose.theta) > random_angle/2):
					msg.angular.z = 0
					pub.publish(msg)
				else:
					msg.angular.z = 0.3
					msg.linear.x = 0
				pub.publish(msg)
				print("rotate more than half, second half")
			first_run = True

def odom_callback(msg):
	# get pose = (x, y, theta) from odometry topic
	global pose
	quarternion=[msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
	pose.theta = yaw
	pose.x = msg.pose.pose.position.x
	pose.y = msg.pose.pose.position.y

	trajectory.append([pose.x, pose.y])

def visualise():
	_,ax = plt.subplots(1)
	ax.set_aspect('equal')

	trajectory = np.loadtxt("trajectory.csv", delimiter=',')
	plt.plot(trajectory[:,0], trajectory[:,1], linewidth =2)
		
	plt.xlim(-3,3)
	plt.ylim(-3,3)
	plt.show()

def plot():
	np.savetxt('trajectory.csv', np.array(trajectory), fmt='%f', delimiter=',')
	visualise()

if __name__ == '__main__':
	main()
	plot()
<<<<<<< HEAD
=======
	
>>>>>>> a0af553250449e04bde3830bb29e5d250d6c58d8

