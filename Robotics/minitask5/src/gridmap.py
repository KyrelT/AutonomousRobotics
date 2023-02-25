#!/usr/bin/env python

import numpy as np
import cv2
import copy
from bresenhamline import *

class GridMap:
	def __init__(self, X_lim, Y_lim, resolution):
		# Initialise the variables
		self.X_lim = X_lim
		self.Y_lim = Y_lim
		self.resolution = resolution

		# Make the size of matrix based on the resolution and world size
		self.x = np.arange(start = X_lim[0], stop = X_lim[1] + resolution, step = resolution)
		self.y = np.arange(start = Y_lim[0], stop = Y_lim[1] + resolution, step = resolution)
		
		# probability matrix with -1, unknown cell:
		self.probability_matrix = np.full(shape = (len(self.x), len(self.y)), fill_value = -1)
	

	# Log Odds Trick
	def log_odds(self, probability):
		return np.log(probability / (1 - probability))
	

	# Change the coordinate of world to grid
	def to_grid(self, x_coord, y_coord, x_size, y_size, x_lim, y_lim, resolution):

		x = int((x_coord - x_lim) / resolution)
		y = int((y_coord - y_lim) / resolution)
		if x >= x_size:
			x = x_size - 1
		elif x < 0:
			x = 0

		if y >= y_size:
			y = y_size - 1
		elif y < 0:
			y = 0

		return (x,y)
	

	# Change the coordinate of grid to world
	def to_world(self, x_coord, y_coord, x_size, y_size, x_lim, y_lim, resolution):
		x = (x_coord * resolution) + x_lim + (resolution / 2)
		y = (y_coord * resolution) + y_lim + (resolution / 2)
		
		if x > x_size[1]:
			x = x_size[1]
		elif x < x_size[0]:
			x = x_size[0]

		if y > y_size[1]:
			y = y_size[1]
		elif y < y_size[0]:
			y = y_size[0]

		return (x,y)
	

	# Update the grid cell based on the probability
	def update(self, x, y, probability):
		if x >= len(self.x):
			x = len(self.x) - 1
		elif x < 0:
			x = 0
		if y >= len(self.y):
			y =len(self.y) - 1
		elif y < 0:
			y = 0
		# update probability matrix using inverse sensor model, log odds trick
		# check if the x and y coordinate is newly found
		if self.probability_matrix[x][y] == -1:
			# if it is newly found, then equals to probability
			self.probability_matrix[x][y] = self.log_odds(probability)
		else:
			# if not then add the probability to previous probability.
			self.probability_matrix[x][y] += self.log_odds(probability)


	# Retrieve back the probability, reverse of log odd tricks
	def retrieve_p(self, probability_matrix):
		return 1 - 1 / (1 + np.exp(self.probability_matrix))


	# Size of the matrix shape
	def get_shape(self):
		return np.shape(self.probability_matrix)


	# Change the probability matrix to coloured image
	def to_BGR_image(self):
		image_matrix = self.probability_matrix.copy()
		gray_image = 1 - self.retrieve_p(image_matrix)

		rgb_image = np.repeat(a = gray_image[:,:,np.newaxis],repeats = 3,axis = 2)
		
		return rgb_image


	# Check if the pixel is within map
	def check_pixel(self, x, y):
		if x >= 0 and x < self.get_shape()[0] and y >= 0 and y < self.get_shape()[1]:
			return True 
		else:
			return False
	

	# Set the colour of pixel grid
	def set_pixel_color(self, bgr_image, x, y, color):
		if x < 0 or y < 0 or x >= bgr_image.shape[0] or y >= bgr_image.shape[1]:
			return 

		if color == 'GREEN':
			bgr_image[x, y, 0] = 0.0
			bgr_image[x, y, 1] = 1.0
			bgr_image[x, y, 2] = 0.0
		elif color == 'RED':
			bgr_image[x, y, 0] = 0.0
			bgr_image[x, y, 1] = 0.0
			bgr_image[x, y, 2] = 1.0


	# Check the surrounding pixel
	def find_neighbours(self, x, y):
		X_neighbours = []
		Y_neighbours = []

		if self.check_pixel(x + 1, y):
			X_neighbours.append(x + 1)
			Y_neighbours.append(y)
		
		if self.check_pixel(x + 1, y + 1):
			X_neighbours.append(x + 1)
			Y_neighbours.append(y + 1)
		
		if self.check_pixel(x + 1, y - 1):
			X_neighbours.append(x + 1)
			Y_neighbours.append(y - 1)
		
		if self.check_pixel(x, y + 1):
			X_neighbours.append(x)
			Y_neighbours.append(y + 1)

		if self.check_pixel(x, y - 1):
			X_neighbours.append(x)
			Y_neighbours.append(y - 1)

		if self.check_pixel(x - 1, y):
			X_neighbours.append(x - 1)
			Y_neighbours.append(y)

		if self.check_pixel(x - 1, y + 1):
			X_neighbours.append(x - 1)
			Y_neighbours.append(y + 1)

		if self.check_pixel(x - 1, y - 1):
			X_neighbours.append(x - 1)
			Y_neighbours.append(y - 1)

		return zip(X_neighbours, Y_neighbours)


	# Check the surrounding pixel
	def check_surrounding(self, x, y, matrix, cell):
		result_list = []

		if self.check_pixel(x + 1, y):
			if matrix[x + 1][y] == cell:
				result_list.append(True)
			else:
				result_list.append(False)
		
		if self.check_pixel(x + 1, y + 1):
			if matrix[x + 1][y + 1] == cell:
				result_list.append(True)
			else:
				result_list.append(False)
		
		if self.check_pixel(x + 1, y - 1):
			if matrix[x + 1][y - 1] == cell:
				result_list.append(True)
			else:
				result_list.append(False)
	
		if self.check_pixel(x, y + 1):
			if matrix[x][y + 1] == cell:
				result_list.append(True)
			else:
				result_list.append(False)
		if self.check_pixel(x, y - 1):
			if matrix[x][y - 1] == cell:
				result_list.append(True)
			else:
				result_list.append(False)
		if self.check_pixel(x - 1, y):
			if matrix[x - 1][y] == cell:
				result_list.append(True)
			else:
				result_list.append(False)

		if self.check_pixel(x - 1, y + 1):
			if matrix[x - 1][y + 1] == cell:
				result_list.append(True)
			else:
				result_list.append(False)

		if self.check_pixel(x - 1, y - 1):
			if matrix[x - 1][y - 1] == cell:
				result_list.append(True)
			else:
				result_list.append(False)

		return result_list


	# Check if the grid has obstacle around it
	def check_obstacle(self, x, y, matrix):
		result_list = []
		for i in range(-3, 4):
			for j in range(-3, 4):
				if self.check_pixel(x + i, y + j):
					if matrix[x + i][y + j] > 60:
						result_list.append(False)
					else:
						result_list.append(True)

		return result_list


	# Check if the obstacles is small and not walls, (Like guessing obstacles)
	def smart_obstacles(self, x, y, matrix):
		result_list = []
		
		if self.check_pixel(x, y - 3):
			if matrix[x][y - 3] == 0:
				result_list.append(True)
			else:
				result_list.append(False)
		
		if self.check_pixel(x, y + 3):
			if matrix[x][y + 3] == 0:
				result_list.append(True)
			else:
				result_list.append(False)

		if self.check_pixel(x - 3, y):
			if matrix[x - 3][y] == 0:
				result_list.append(True)
			else:
				result_list.append(False)

		if self.check_pixel(x + 3, y):
			if matrix[x + 3][y] == 0:
				result_list.append(True)
			else:
				result_list.append(False)

		return result_list

	# Add the surrounding grid to the list
	def add_to_list(self, x, y, checklist):
		for i in range(-4, 5):
			for j in range(-4, 5):
				if self.check_pixel(x + i, y + j):
					checklist.append([x + i, y + j])
					
