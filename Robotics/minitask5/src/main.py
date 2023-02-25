#!/usr/bin/env python

import rospy
import numpy as np
import random
import copy

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Pose2D
from navigation import *
from gridmap import *
from callbacks import *

if __name__ == '__main__':
	try:
		# Init ROS node
		rospy.init_node('gmapping_node', anonymous = False)

		# limits for X and Y (size)
		X_lim = [-2.0,6.4]
		Y_lim = [-4.2,4.6]
		resolution = 0.10

		# Create grid map 
		gridMap = GridMap(X_lim, Y_lim, resolution)
		cv2.namedWindow("grid_map", 1)
		cv2.namedWindow("Original", 1)
		cv2.namedWindow("Masked", 1)
		#cv2.namedWindow("Robot Vision", 1)

		mymap = OccupancyGrid(header = Header(seq=0, stamp = rospy.Time.now(), frame_id="map"), info = MapMetaData(width=len(gridMap.y), height=len(gridMap.x), resolution=resolution ,map_load_time=rospy.Time.now()))
		
		# Call the python files
		callbacks = OdomLaserScan()

		navigation = navigation()

		map_pub=rospy.Publisher('/map',OccupancyGrid,queue_size=1)

		rate = rospy.Rate(10)

		# Initialise variables
		previous_x = 10000
		previous_y = 10000
		false_navigation = []
		green_count = 0
		green_list = []
		red_count = 0
		red_list = []
		
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
			
			# Change the grid map to coloured image to visualise
			bgr_image = gridMap.to_BGR_image()

			# Mark robot position to red
			gridMap.set_pixel_color(bgr_image, x1, y1, 'RED')
			
			# Make the red robot position bigger
			for (x, y) in gridMap.find_neighbours(x1, y1):
				gridMap.set_pixel_color(bgr_image, x, y, 'RED')
			
			# Mark the robot's 360 degree laser maximum range as green
			for (x, y) in zip(X2,Y2):
				gridMap.set_pixel_color(bgr_image, x, y, 'GREEN')

			resized_image = cv2.resize(src = bgr_image, dsize = (500, 500), interpolation = cv2.INTER_AREA)

			rotated_image = cv2.rotate(src = resized_image, rotateCode = cv2.ROTATE_180)

			cv2.imshow("grid_map", rotated_image)
			cv2.waitKey(1)	
			
			# Initialise variables each loop to ensure it always reset
			data_occupancy = gridMap.probability_matrix.copy()
			mymap.data = []
			lowest_distance = 1000
			unknown_count = 0
			empty_list = []
			obstacle_list= []

			# Change the negative known cell to 0 and more than 100 cell to 100
			data_occupancy[np.logical_and(data_occupancy != -1, data_occupancy < 0)] = 0
			data_occupancy[data_occupancy > 100] = 100

			for i in range(0, len(gridMap.x) - 1):
				for j in range(0, len(gridMap.y) - 1):
					if data_occupancy[i][j] == -1:
						unknown_count += 1
						# Check if the unknown cell grid has obstacle around? to ensure the navigation wont go too near a wall
						# But sometimes it will still be too near, so check if the navigation is in a list of coordinate that is not able to be navigated
						if all(gridMap.check_obstacle(i, j, data_occupancy)) and [i, j] not in false_navigation:
							# Get the nearest unknown cell that has an empty cell beside to have a smarter navigation instead of randomisation
							if any(gridMap.check_surrounding(i, j, data_occupancy, 0)):
								distance = math.sqrt((x1 - i)**2 + (y1 - j)**2)
								if distance < lowest_distance:
									nav_grid_coord = [i, j]
									lowest_distance = distance

							empty_list.append([i,j])

					# Check if the grid has 60% chance of being an obstacle
					elif data_occupancy[i][j] > 60:
						# Check if the grid obstacle is not too big so that it wont navigate a wall, only small objects
						if all(gridMap.smart_obstacles(i, j, data_occupancy)) and [i, j] not in false_navigation:
							obstacle_list.append([i, j])
					
					mymap.data.append(data_occupancy[i][j])

			# Initalise the navigation to middle
			if callbacks.middle is True:
				x_world, y_world = gridMap.to_world(len(gridMap.x) / 2 + 5, len(gridMap.y) / 2 + 1, X_lim, Y_lim, X_lim[0], Y_lim[0], resolution)
				navigation.moveToCoordinates(x_world, y_world)
				callbacks.middle = False

				navigation.goalStarted = True
				t = rospy.Time.now().to_sec()
			else:

				# if unknown cell is more than 250 and no objects found, then start navigation goal
				if unknown_count > 250 :
					if navigation.goalStarted is False and callbacks.greenbeaconFound is False and callbacks.redHydrantFound is False and callbacks.blueTilesFound is False and callbacks.blueTilesMove is False and callbacks.obstacle_found is False:
						print("unknown cell left: " + str(unknown_count))
						# Check if the previous navigation failed due to walls
						if previous_x == x1 and previous_y == y1:
							navigation.same_navigation = True
							navigation.ac.cancel_goal()
							print("stopping")
							false_navigation.append(nav_grid_coord)

						# If no problem previously, then go to the nearest unknown grid
						if navigation.same_navigation is False:
							previous_x = x1
							previous_y = y1
							x_world, y_world = gridMap.to_world(nav_grid_coord[0] + 5, nav_grid_coord[1] + 1, X_lim, Y_lim, X_lim[0], Y_lim[0], resolution)
							navigation.moveToCoordinates(x_world, y_world)
							navigation.goalStarted = True
							t = rospy.Time.now().to_sec()
							print("navigate to nearest unknown cell")

						# If navigation failed previously then randomised a grid position			
						else:
							number = random.randint(0, len(empty_list) - 1)
							nav_grid_coord = empty_list[number]							
							previous_x = x1
							previous_y = y1
							x_world, y_world = gridMap.to_world(nav_grid_coord[0] + 5, nav_grid_coord[1] + 1, X_lim, Y_lim, X_lim[0], Y_lim[0], resolution)
							navigation.moveToCoordinates(x_world, y_world)
							navigation.goalStarted = True
							t = rospy.Time.now().to_sec()
							navigation.same_navigation = False
							print("navigate to randomised unknown cell")

						print(nav_grid_coord)

				# if less than 250 and no objects found then go to the obstacle that is over 60% to find objects
				elif unknown_count < 250 and navigation.goalStarted is False and callbacks.greenbeaconFound is False and callbacks.redHydrantFound is False and callbacks.blueTilesFound is False and callbacks.blueTilesMove is False and callbacks.obstacle_found is False:
					number = random.randint(0, len(obstacle_list) - 1)
					nav_grid_coord = obstacle_list[number]	

					# Check if the previous navigation failed due to walls
					if previous_x == x1 and previous_y == y1:
						navigation.same_navigation = True
						navigation.ac.cancel_goal()
						print("stopping")
						false_navigation.append(nav_grid_coord)

					x_world, y_world = gridMap.to_world(nav_grid_coord[0] + 10, nav_grid_coord[1] + 6, X_lim, Y_lim, X_lim[0], Y_lim[0], resolution)
					navigation.moveToCoordinates(x_world, y_world)
					navigation.goalStarted = True
					t = rospy.Time.now().to_sec()
					print("navigate and find objects")


			# Cancel the goal and start a new goal every 7 seconds or when object is found
			if navigation.goalStarted is True and (rospy.Time.now().to_sec() - t > 7 or navigation.same_navigation is True or callbacks.greenbeaconFound is True or callbacks.redHydrantFound is True or callbacks.blueTilesFound is True):
			
				navigation.goalStarted = False
				navigation.ac.cancel_goal()
				print("stopping")

			# Announce the count of object when found
			if callbacks.announceCount is True:
				if callbacks.redStarted is True:
					if [x1, y1] not in red_list: # Check if the robot position is in the red object list
						red_count += 1
						print("Red Object #" + str(red_count) + " found")
						# Add the robot surrounding coordinate to the red object list to avoid announcing same red object
						gridMap.add_to_list(x1, y1, red_list)
				elif callbacks.greenStarted is True:
					if [x1, y1] not in green_list: # Check if the robot position is in the green object list
						green_count += 1
						print("Green Object #" + str(green_count) + " found")
						# Add the robot surrounding coordinate to the green object list to avoid announcing same green object
						gridMap.add_to_list(x1, y1, green_list)
				callbacks.announceCount = False


			rate.sleep()

		rospy.spin()

	except rospy.ROSInterruptException:
		print('\r\nSIMULATION TERMINATED!')
		map_pub.publish(mymap)
		cv2.destroyAllWindows()
		navigation.ac.cancel_goal()
		print("stopping")




