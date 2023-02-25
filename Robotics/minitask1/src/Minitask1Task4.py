#!/usr/bin/env python2

import rospy
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import tf


class TurtlebotDriving:

	def __init__(self):
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.pose = Pose2D()
		self.trajectory = list()
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

	def move(self):
		x_init = self.pose.x
		y_init = self.pose.y

		while math.sqrt((self.pose.x - x_init)**2 + (self.pose.y - y_init)**2) < 1 and not rospy.is_shutdown():
			velocity = Twist()
			velocity.linear.x = 0.1
			self.pub.publish(velocity)
		print("odom: x=" + str(self.pose.x) + ";  y=" + str(self.pose.y) + ";  theta=" + str(self.pose.theta))	
			
	def turn(self):
		a_init = self.pose.theta

		while (max(self.pose.theta - a_init, a_init - self.pose.theta) < (math.pi/2)) and not rospy.is_shutdown():
			velocity = Twist()
			velocity.angular.z = 0.1
			self.pub.publish(velocity)
		print("odom: x=" + str(self.pose.x) + ";  y=" + str(self.pose.y) + ";  theta=" + str(self.pose.theta))		

	def stop(self):
		np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')
		self.visualise()
		while not rospy.is_shutdown():
			velocity = Twist()
			velocity.linear.x = 0
			velocity.angular.z = 0
			self.pub.publish(velocity)
	
	def visualise(self):
		_,ax = plt.subplots(1)
		ax.set_aspect('equal')

		self.trajectory = np.loadtxt("trajectory.csv", delimiter=',')
		plt.plot(self.trajectory[:,0], self.trajectory[:,1], linewidth =2)
		
		plt.xlim(-0.5,1.5)
		plt.ylim(-0.5,1.5)
		plt.show()

	def odom_callback(self, msg):
		# get pose = (x, y, theta) from odometry topic
		quarternion=[msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
		self.pose.theta = yaw
		self.pose.x = msg.pose.pose.position.x
		self.pose.y = msg.pose.pose.position.y

		self.trajectory.append([self.pose.x, self.pose.y])


if __name__=="__main__":
	rospy.init_node('TurtlebotDriving')
	a = TurtlebotDriving()
	a.move()
	a.turn()
	a.move()
	a.turn()
	a.move()
	a.turn()
	a.move()
	a.turn()
	a.stop()
	rospy.spin()

