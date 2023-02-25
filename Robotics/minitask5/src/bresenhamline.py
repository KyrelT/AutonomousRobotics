#!/usr/bin/env python

import numpy as np

def bresenham(x1, y1, x2, y2, max_x, max_y):
	# Output pixels
	X_point = []
	Y_point = []

	# Differentiation of the line to know the gradient of line
	dx = np.abs(x2 - x1)
	dy = np.abs(y2 - y1)

	# Determine how steep the line is
	if dy > dx:
		# Rotate line
		dx, dy = dy, dx
		interchange = True

	else:
		interchange = False
	
	# Calculate error
	error_x = np.sign(x2 - x1)
	error_y = np.sign(y2 - y1)

	x = x1
	y = y1

	# mark output pixels
	X_point.append(x)
	Y_point.append(y)
	
	# Determine whether x or y or both need to be changed
	A = 2 * dy
	B = 2 * (dy - dx)
	error = 2 * dy - dx	

	# Put every coordinate in correct pixel grid, exactly on the grid
	for i in range(1, dx):
		if error < 0:
			if interchange:
				y += error_y
			else:
				x += error_x

			error = error + A

		else:
			y += error_y
			x += error_x
			error = error + B
		# mark output pixels
		X_point.append(x)
		Y_point.append(y)

	
	# Return the X and Y coordinate
	return zip(X_point, Y_point)


