#!/usr/bin/env python

import rospy

import numpy as np
import random
import copy

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Pose2D
#from explore import *
#from navigation import *
from gridmap import *
from callbacks import *

if __name__ == '__main__':
	try:
		# Init ROS node
		rospy.init_node('gmapping_node', anonymous = False)

		
		X_lim = [-8,8]
		Y_lim = [-6,5.5]
		resolution = 0.10

		# Create grid map 
		gridMap = GridMap(X_lim, Y_lim, resolution)
		cv2.namedWindow("grid_map", 1)

		mymap = OccupancyGrid(header = Header(seq=0, stamp = rospy.Time.now(), frame_id="map"), info = MapMetaData(width=len(gridMap.y), height=len(gridMap.x), resolution=resolution ,map_load_time=rospy.Time.now()))
		
		# Initialise the Subscriber node
		callbacks = OdomLaserScan()

		map_pub=rospy.Publisher('/map',OccupancyGrid,queue_size=1)

		rate = rospy.Rate(10)
		
		# Main loop
		while not rospy.is_shutdown():
			# Get laser scan message
			distance = callbacks.distances
			angle = callbacks.angles

			# Get odometry message
			pose_x = callbacks.pose.x
			pose_y = callbacks.pose.y
			pose_theta = callbacks.pose.theta

			
			# Laser distance
			laserdistances_x, laserdistances_y = callbacks.distance_scan_xy(distance, angle, pose_x, pose_y, pose_theta)

			# Robot position on gridmoveXCoord map using to_grid
			x1, y1 = gridMap.to_grid(pose_x, pose_y, len(gridMap.x), len(gridMap.y), X_lim[0], Y_lim[0], resolution)
			
			X2 = []
			Y2 = []

			for (dist_x, dist_y, dist) in zip(laserdistances_x, laserdistances_y, distance):
				
				# Furthest laser position on grid map using to_grid
				x2, y2 = gridMap.to_grid(dist_x, dist_y, len(gridMap.x), len(gridMap.y), X_lim[0], Y_lim[0], resolution)

				# draw a discrete line of free pixels, [robot position -> laser hit spot)
				for (x_bres, y_bres) in bresenham(x1, y1, x2, y2, gridMap.probability_matrix.shape[0], gridMap.probability_matrix.shape[1]):

					gridMap.update(x = x_bres, y = y_bres, probability = 0.3)

				# mark laser hit spot as ocuppied (if exists ie distance is less than max laser range)
				if dist < callbacks.max:
					
					gridMap.update(x = x2, y = y2, probability = 0.9)
					
				# For CV2
				X2.append(x2)
				Y2.append(y2)
			
			bgr_image = gridMap.to_BGR_image()

			# Mark robot pos to red
			gridMap.set_pixel_color(bgr_image, x1, y1, 'RED')
			for (x, y) in gridMap.find_neighbours(x1, y1):
				gridMap.set_pixel_color(bgr_image, x, y, 'RED')
			
			for (x, y) in zip(X2,Y2):
				gridMap.set_pixel_color(bgr_image, x, y, 'GREEN')

			resized_image = cv2.resize(src = bgr_image, dsize = (500, 500), interpolation = cv2.INTER_AREA)

			rotated_image = cv2.rotate(src = resized_image, rotateCode = cv2.ROTATE_180)

			cv2.imshow("grid_map", rotated_image)
			cv2.waitKey(1)	
			
			data_occupancy = gridMap.probability_matrix.copy()
			mymap.data = []

			for i in range(0, len(gridMap.x) - 1):
				for j in range(0, len(gridMap.y) - 1):
					if data_occupancy[i][j] != -1:
						if data_occupancy[i][j] >= 100:
							data_occupancy[i][j] = 100
						elif data_occupancy[i][j] <= 0:
							data_occupancy[i][j] = 0

					mymap.data.append(data_occupancy[i][j])

			#print()
			#map_pub.publish(mymap)
			rate.sleep()

	except rospy.ROSInterruptException:
		print('\r\nSIMULATION TERMINATED!')
		map_pub.publish(mymap)
		cv2.destroyAllWindows()

