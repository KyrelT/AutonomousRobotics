#!/usr/bin/env python

import numpy as np
import cv2
import copy
from bresenhamline import *
import callbacks

class GridMap:
	def __init__(self, X_lim, Y_lim, resolution):
		self.X_lim = X_lim
		self.Y_lim = Y_lim
		self.resolution = resolution
		self.callbacks = callbacks.OdomLaserScan()

		self.x = np.arange(start = X_lim[0], stop = X_lim[1] + resolution, step = resolution)
		self.y = np.arange(start = Y_lim[0], stop = Y_lim[1] + resolution, step = resolution)
		
		# probability matrix in log-odds scale:
		self.probability_matrix = np.full(shape = (len(self.x), len(self.y)), fill_value = -1)
	
	def log_odds(self, probability):
		return np.log(probability / (1 - probability))
	
	# x_lim and y_lim is the lower bound of x and y
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
	
	def to_world(self, x_coord, y_coord, x_size, y_size, x_lim, y_lim, resolution):
		x = (x_coord * resolution) + x_lim
		y = (y_coord * resolution) + y_lim
		
		if x > x_size[1]:
			x = x_size[1]
		elif x < x_size[0]:
			x = x_size[0]

		if y > y_size[1]:
			y = y_size[1]
		elif y < y_size[0]:
			y = y_size[0]

		return (x,y)
	
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


	def retrieve_p(self, probability_matrix):
		return 1 - 1 / (1 + np.exp(self.probability_matrix))


	def get_shape(self):
		return np.shape(self.probability_matrix)


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

