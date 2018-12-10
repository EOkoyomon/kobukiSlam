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
import pickle


def matprint(mat, fmt="g"):
	col_maxes = [max([len(("{:"+fmt+"}").format(x)) for x in col]) for col in mat.T]
	for x in mat:
		for i, y in enumerate(x):
			print(("{:"+str(col_maxes[i])+fmt+"}").format(y), end="  ")
		print("")

# read point cloud
pcd = read_point_cloud('low.pcd')
points = np.asarray(pcd.points)
x_values, y_values, z_values = points[:, 0], points[:, 1], points[:, 2]

# get min and max x and y values to construct grid
x_min, x_max = np.min(x_values), np.max(x_values)
y_min, y_max = np.min(y_values), np.max(y_values)

grid_min = min(x_min, y_min)
grid_max = max(x_max, y_max)

space_size = 0.1

grid_size = (grid_max - grid_min) / space_size
grid_size = int(grid_size)


# construct grid
x_dir = np.linspace(grid_min, grid_max, grid_size)
y_dir = np.linspace(grid_min, grid_max, grid_size)

grid_x = np.searchsorted(x_dir, x_values)
grid_y = np.searchsorted(y_dir, y_values)

grid = np.zeros((grid_size, grid_size))
for i, point in enumerate(zip(grid_x, grid_y)):
	if z_values[i] >= 0.1:
		grid[point] += 1

# matprint(grid)

# find spot for initial position
init_pos = [1, -1, -1.35669111]
init_x = np.searchsorted(x_dir, init_pos[0])
init_y = np.searchsorted(y_dir, init_pos[1])

# find start spot to return to
t_x = np.searchsorted(x_dir, 0)
t_y = np.searchsorted(y_dir, 0)

# construct graph
start_node = init_x*grid_size + init_y
end_node = t_x*grid_size + t_y

graph = Graph()
cost_func = lambda u, v, e, prev_e: e['cost']

threshold_points = 0

# the node number will be grid_size*row_num + column_num
for i, row in enumerate(grid):
	for j, val in enumerate(row):
		node = grid_size*i + j

		if grid[i, j] <= threshold_points:
			# look at the adjacent nodes, if they are empty, add an edge
			for m in (i-1, i, i+1):
				for n in (j-1, j, j+1):
					if m >= 0 and m < grid_size and n >= 0 and n < grid_size:
						if grid[m, n] <= threshold_points:
							adj_node = grid_size*m + n
							graph.add_edge(node, adj_node, {'cost': 1})


# find nearest unoccupied start node
start = min(list(graph.keys()), key=lambda x:abs(x-start_node))
end = min(list(graph.keys()), key=lambda x:abs(x-end_node))
shortest_path = find_path(graph, start, end, cost_func=cost_func)

# load orientation
with open('duck_vid-old_c.pickle', 'rb') as r:
	explore = pickle.load(r)
	orientation = pickle.load(r)

# explore = np.array(explore)
# exp_x = np.searchsorted(x_dir, explore[:, 0])
# exp_y = np.searchsorted(y_dir, explore[:, 1])
# for point in zip(exp_x, exp_y):
# 	grid[point] = -1

grid[t_x, t_y] = -2 # initial starting spot
grid[init_x, init_y] = -3 # where we stopped

for n in shortest_path.nodes[:-1]:
	grid[n // grid_size, n % grid_size] = -4

# print the grid
def matprint2(mat, fmt='g'):
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


matprint2(grid)

def rotation_angle(current, desired):
	dot = current[0]*desired[0] + current[1]*desired[1]      # dot product between [x1, y1] and [x2, y2]
	det = current[0]*desired[1] - current[1]*desired[0]      # determinant
	angle = np.arctan2(det, dot)
	return np.rad2deg(angle)

'''
____________
_0_|_1_|_2_|
_3_|_4_|_5_|
_6_|_7_|_8_|
'''

instructions = []

s = shortest_path.nodes[0]
current_orientation = orientation
for n in shortest_path.nodes[1:]:

	s_x, s_y = s // grid_size, s % grid_size
	n_x, n_y = n // grid_size, n % grid_size

	# assume we start at 4

	if n_x < s_x and n_y < s_y:
		# 0, (-0.71, 0.71)
		target = [-0.71, 0.71]
		distance = space_size * np.sqrt(2)
	elif n_x == s_x and n_y < s_y:
		# 3, (-1, 0)
		target = [-1, 0]
		distance = space_size
	elif n_x > s_x and n_y < s_y:
		# 6, (-0.71, -0.71)
		target = [-0.71, -0.71]
		distance = space_size * np.sqrt(2)
	elif n_x < s_x and n_y == s_y:
		# 1, (0, 1)
		target = [0, 1]
		distance = space_size
	elif n_x > s_x and n_y == s_y:
		# 7, (0, -1)
		target = [0, -1]
		distance = space_size
	elif n_x < s_x and n_y > s_y:
		# 2 (0.71, 0.71)
		target = [0.71, 0.71]
		distance = space_size * np.sqrt(2)
	elif n_x == s_x and n_y > s_y:
		# 5 (1, 0)
		target = [1, 0]
		distance = space_size
	elif n_x > s_x and n_y > s_y:
		# 8 (0.71, -0.71)
		target = [0.71, -0.71]
		distance = space_size * np.sqrt(2)
	
	# how much we need to rotate in this square, positive is counterclockwise
	rotation = rotation_angle(current_orientation, target)
	current_orientation = target

	print('rotate:', rotation, 'drive', distance, '(%d, %d) -> (%d, %d)' % (s_x, s_y, n_x, n_y))
	s = n

	instructions.append((rotation, distance))










