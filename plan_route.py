'''

We have a list of points (obstacles), current position

- divide 3d space into a grid
- fill each grid space with the points
- if the grid space has less than n points, it is "empty"
- construct graph out of empty adjacent grid spaces
- run dijkstras and get shortest path

'''

import numpy as np
from open3d import *
from dijkstar import Graph, find_path

# print the matrix
def matprint(mat, fmt="g"):
	col_maxes = [max([len(("{:"+fmt+"}").format(x)) for x in col]) for col in mat.T]
	for x in mat:
		for i, y in enumerate(x):
			print(("{:"+str(col_maxes[i])+fmt+"}").format(y), end="  ")
		print("")


# visualize occupancy grid
def matprint2(mat, threshold_points, fmt='g'):
	col_maxes = [max([len(("{:"+fmt+"}").format(x)) for x in col]) for col in mat.T]
	for j, x in enumerate(mat):
		for i, y in enumerate(x):
			# if y == -1:
			# 	n = 'P'
			if y == -2:
				n = 'S'
			elif y == -3:
				n = 'E'
			elif y == -4:
				n = 'P'
			else:
				n = int(y>threshold_points)
				if n == 1:
					n = 'X'
				else:
					n = ' '
					
			print(("{:"+str(col_maxes[i])+'s'+"}").format(n), end="  ")
		print("")

def rotation_angle(current, desired):
	dot = current[0]*desired[0] + current[1]*desired[1]      # dot product between [x1, y1] and [x2, y2]
	det = current[0]*desired[1] - current[1]*desired[0]      # determinant
	angle = np.arctan2(det, dot)
	return np.rad2deg(angle)

def plan_route(pcd_name, position_name):
	data = []
	with open(position_name, 'r'):
		for line in position_name:
			data.append(float(line))
	plan_route1(pcd_name, [data[0], data[1]], data[3])

def plan_route1(pcd_name, end_position, end_orientation, space_size=0.1, display=False):

	# read point cloud
	pcd = read_point_cloud(pcd_name)
	points = np.asarray(pcd.points)
	x_values, y_values, z_values = points[:, 0], points[:, 1], points[:, 2]

	# get min and max x and y values to construct grid
	x_min, x_max = np.min(x_values), np.max(x_values)
	y_min, y_max = np.min(y_values), np.max(y_values)

	grid_min = min(x_min, y_min)
	grid_max = max(x_max, y_max)

	# grid dimensions
	grid_size = (grid_max - grid_min) / space_size
	grid_size = int(grid_size)


	# construct grid
	x_dir = np.linspace(grid_min, grid_max, grid_size)
	y_dir = np.linspace(grid_min, grid_max, grid_size)

	# grid indices of points
	grid_x = np.searchsorted(x_dir, x_values)
	grid_y = np.searchsorted(y_dir, y_values)

	grid = np.zeros((grid_size, grid_size))

	# fill grid
	for i, point in enumerate(zip(grid_x, grid_y)):
		if z_values[i] >= 0.1:
			grid[point] += 1

	# grid index of end position
	end_x = np.searchsorted(x_dir, end_position[0])
	end_y = np.searchsorted(y_dir, end_position[1])

	# grid index of 0,0
	start_x = np.searchsorted(x_dir, 0)
	start_y = np.searchsorted(y_dir, 0)

	# get node numbers of start and end grid spaces
	end_node = end_x*grid_size + end_y
	start_node = start_x*grid_size + start_y

	# construct graph
	graph = Graph()
	cost_func = lambda u, v, e, prev_e: e['cost']

	# threshold for considering a square empty
	threshold_points = 0

	# the node number will be grid_size*row_num + column_num
	for i, row in enumerate(grid):
		for j, val in enumerate(row):
			node = grid_size*i + j

			if grid[i, j] <= threshold_points:
				# look at the adjacent nodes, if they are empty, add an edge
				for m in (i-1, i, i+1):
					for n in (j-1, j, j+1):
						if not (m == i and n == j) and m >= 0 and m < grid_size and n >= 0 and n < grid_size:
							if grid[m, n] <= threshold_points:
								adj_node = grid_size*m + n
								if abs(m-i)+abs(m-j) == 2: # weight diagonal nodes higher
									graph.add_edge(node, adj_node, {'cost': 2})
								else:
									graph.add_edge(node, adj_node, {'cost': 1})


	# find nearest unoccupied start node
	start = min(list(graph.keys()), key=lambda x:abs(x-start_node))
	end = min(list(graph.keys()), key=lambda x:abs(x-end_node))

	# from the ending node, find shortest path back to starting node
	shortest_path = find_path(graph, end, start, cost_func=cost_func)

	if display:
		# printing utility
		temp = np.copy(grid)
		temp[start_x, start_y] = -2 # initial starting spot
		temp[end_x, end_y] = -3 # where we stopped

		for n in shortest_path.nodes[1:-1]:
			temp[n // grid_size, n % grid_size] = -4

		matprint2(temp, threshold_points)


	'''
	____________
	_8_|_7_|_6_|
	_5_|_4_|_3_|
	_2_|_1_|_0_|
	'''

	instructions = []

	# changing the orientation to radians
	if end_orientation >= 0:
		end_orientation = np.pi * end_orientation
	else:
		end_orientation = 2*np.pi + (end_orientation * np.pi)


	s = shortest_path.nodes[0]
	current_orientation = [np.cos(end_orientation), np.sin(end_orientation)]
	for n in shortest_path.nodes[1:]:

		s_x, s_y = s // grid_size, s % grid_size
		n_x, n_y = n // grid_size, n % grid_size

		# assume we start at 4

		if n_x < s_x and n_y < s_y:
			# 8, (-0.71, -0.71)
			target = [-0.71, -0.71]
			distance = space_size * np.sqrt(2)
		elif n_x == s_x and n_y < s_y:
			# 5, (0, -1)
			target = [0, -1]
			distance = space_size
		elif n_x > s_x and n_y < s_y:
			# 2, (0.71, -0.71)
			target = [0.71, -0.71]
			distance = space_size * np.sqrt(2)
		elif n_x < s_x and n_y == s_y:
			# 7, (-1, 0)
			target = [-1, 0]
			distance = space_size
		elif n_x > s_x and n_y == s_y:
			# 1, (1, 0)
			target = [1, 0]
			distance = space_size
		elif n_x < s_x and n_y > s_y:
			# 6 (-0.71, 0.71)
			target = [-0.71, 0.71]
			distance = space_size * np.sqrt(2)
		elif n_x == s_x and n_y > s_y:
			# 3 (0, 1)
			target = [0, 1]
			distance = space_size
		elif n_x > s_x and n_y > s_y:
			# 0 (0.71, 0.71)
			target = [0.71, 0.71]
			distance = space_size * np.sqrt(2)
		
		# how much we need to rotate in this square, positive is counterclockwise
		rotation = rotation_angle(current_orientation, target)
		current_orientation = target

		if display:
			print('rotate:', rotation, 'drive', distance, '(%d, %d) -> (%d, %d)' % (s_x, s_y, n_x, n_y))
		s = n

		instructions.append((rotation, distance))

	return instructions










