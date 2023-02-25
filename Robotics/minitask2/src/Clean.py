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

class Turtlebot:
	def main(self):
    		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    		self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
    		self.pose = Pose2D()
    		self.trajectory = list()
    		self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
    		self.wall_found = False
    		self.obstacle_found = False
    		self.first_run = True
    		self.msg = Twist()
    		rospy.spin()


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

    		if self.wall_found is False:
        		self.obstacles_avoid(regions)
    		else:
        		self.wall_following(regions)


	def obstacles_avoid(self, regions):
	    linear_x = 0
	    angular_z = 0
	    state_description = ''

	    if regions['frontleft'] < 0.5 and regions['frontright'] > 0.5:
		state_description = 'obstacles detected (front left)'
		linear_x = 0
		angular_z = -0.3
		self.obstacle_found = True
	    elif regions['frontleft'] > 0.5 and regions['frontright'] < 0.5:
		if regions['rightleft'] < 0.5 and regions['rightright'] < 0.5:
		    state_description = 'wall at right side, wall found'
		    linear_x = 0.3
		    angular_z = 0
		    self.wall_found = True
		else:
		    state_description = 'obstacles detected (front right)'
		    linear_x = 0
		    angular_z = 0.3
		    self.obstacle_found = True
	    elif regions['frontleft'] < 0.5 and regions['frontright'] < 0.5:
		state_description = 'obstacles detected (front)'
		linear_x = 0
		angular_z = 0.3
		self.obstacle_found = True
	    else:
		state_description = 'performing random walk and detecting for green beacon'
		linear_x = 0.3
		angular_z = 0
		self.random_walk()
		self.obstacle_found = False

	    print(state_description)
	    self.msg.linear.x = linear_x
	    self.msg.angular.z = angular_z
	    self.pub.publish(self.msg)


	def wall_following(self, regions):
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
	    self.msg.linear.x = linear_x
	    self.msg.angular.z = angular_z
	    self.pub.publish(self.msg)


	def random_walk(self):
		global x_init
		global y_init
		global a_init
		if self.first_run is True or self.obstacle_found is True and self.wall_found is False: # init everything once it stops
			x_init = self.pose.x
			y_init = self.pose.y
			a_init = self.pose.theta
			self.first_run = False

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


	def odom_callback(self, msg):
	    # get pose = (x, y, theta) from odometry topic
	    quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
		           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
	    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
	    self.pose.theta = yaw
	    self.pose.x = msg.pose.pose.position.x
	    self.pose.y = msg.pose.pose.position.y

	    self.trajectory.append([self.pose.x, self.pose.y])


	def visualise(self):
		_, ax = plt.subplots(1)
		ax.set_aspect('equal')

		self.trajectory = np.loadtxt("trajectory.csv", delimiter=',')
		plt.plot(self.trajectory[:, 0], self.trajectory[:, 1], linewidth=2)

		plt.xlim(-3, 3)
		plt.ylim(-3, 3)
		plt.show()


	def plot(self):
		np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


if __name__ == '__main__':
	rospy.init_node('reading_laser')
        a = Turtlebot()
	while not rospy.is_shutdown():
		a.main()
		a.plot()
	a.visualise()
