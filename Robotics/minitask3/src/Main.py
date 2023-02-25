#!/usr/bin/env python2

import rospy
import math
import random
import numpy as np
import Behaviours
import cv2, cv_bridge
import matplotlib.pyplot as plt
import tf
from numpy import inf, array
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

class Trying:
	def __init__(self):
		self.bh = Behaviours.Behave()
    		self.odom_sub = rospy.Subscriber('odom', Odometry, self.bh.odom_callback)
    		self.pose = Pose2D()
    		self.trajectory = list()
    		self.sub = rospy.Subscriber('/scan', LaserScan, self.bh.callback)
    		self.wall_found = False
    		self.obstacle_found = False
    		self.first_run = True
    		self.msg = Twist()
		self.green_beacon_found = False
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.bh.image_callback)
		cv2.namedWindow("original", 1)
		cv2.namedWindow("masked", 1)
		cv2.namedWindow("robot_see", 1)
    		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('reading_laser')
	a = Trying()
