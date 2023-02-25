#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import tf
import numpy as np
import matplotlib.pyplot as plt

class TurtlebotDriving:

	def __init__(self):
		
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		r = rospy.Rate(10)
		t = rospy.Time.now().to_sec()
		num_secs = 60

		self.pose = Pose2D()
		self.trajectory = list()
		self.sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

		while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t < num_secs:
			
			#Move the robot
			velocity = Twist()
			velocity.linear.x = 0.10
			velocity.angular.z = 0
			for i in range(100):
				self.pub.publish(velocity)
				r.sleep()
			
			#Turn the robot
			velocity.linear.x = 0
			velocity.angular.z = 0.40
			for i in range(40):
				self.pub.publish(velocity)			
				r.sleep()

			#Print the location of the robot
			print("odom: x=" + str(self.pose.x) + ";  y=" + str(self.pose.y) + ";  theta=" + str(self.pose.theta))
						
		velocity.linear.x = 0
		velocity.angular.z = 0	
		self.pub.publish(velocity)


		np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')
		
			
	#Use callback for subscriber node to know the location of robot
	def odom_callback(self, msg):
		# get pose = (x, y, theta) from odometry topic
		quarternion=[msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
		self.pose.theta = yaw
		self.pose.x = msg.pose.pose.position.x
		self.pose.y = msg.pose.pose.position.y

		self.trajectory.append([self.pose.x, self.pose.y])	

	def visualise(self):
		_,ax = plt.subplots(1)
		ax.set_aspect('equal')

		self.trajectory = np.loadtxt("trajectory.csv", delimiter=',')
		plt.plot(self.trajectory[:,0], self.trajectory[:,1], linewidth =2)
		
		plt.xlim(-0.5,1.5)
		plt.ylim(-0.5,1.5)
		plt.show()

		
if __name__=="__main__":
	rospy.init_node('TurtlebotDriving')
	a = TurtlebotDriving()
	a.visualise()

