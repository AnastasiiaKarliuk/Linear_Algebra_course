# to run this module from command line typing 
#
#      > python robot_navigation.py robot_data.json
#
# it reads the starting and the finishing points as well as 
# the list of obstacles from robot_data.json file and then 
# runs find_path() function that has to return the path
#

import sys
import numpy as np
import json




def find_path(start, finish, obstacles=[]):
	start = np.array(start)
	finish = np.array(finish)
	obstacles = [ np.array(ob) for ob in obstacles]

	print("Start:", start)
	print("Finish point:", finish)
	print("Obstacles:", obstacles)

	##  YOUR CODE STARTS HERE  #######################

	# find polyline that do not intersect any obstacle
	# and return it
	

	## the list of vectors you have to return. It should contain the points that the robot 
	#  has to follow avoiding the obstacles.
	polyline = [start, finish]

	##################################################

	return polyline






### these functions may help you to check if your polyline does not intersect the obstacles 

def check_polyline(polyline, obstacles):
	"""this function returns True if the polyline does not intersect obstacles
	Otherwise it returns False
	You can use it to verify your algorithm
	"""
	for obstacle in obstacles:
		for i in range(len(obstacle)):
			obstacle_segment = (obstacle[i-1], obstacle[i]) 
			for j in range(1, len(polyline)):
				path_segment = (polyline[j-1], polyline[j])
				if is_segments_intersect(obstacle_segment, path_segment):
					print("segments intersect:", obstacle_segment, path_segment)
					return False
	return True


def is_segments_intersect(seg_1, seg_2):
	## let's find two line coming through the two points of each segment
	## v = a * v1 + (1 - a) * v2
	## u = b * u1 + (1 - b) * u2
	## lines intersect at u = v, =>  a * v1 + (1 - a) * v2 = b * u1 + (1 - b) * u2
	## or  (v1 - v2) * a + (u2 - u1) * b = u2 - v2
	## 
	## if lines intersect within the given segments, a and b must be strictly between 0 and 1 

	v1, v2 = seg_1
	u1, u2 = seg_2

	M = np.array([v1 - v2, u2 - u1]).T
	if np.linalg.matrix_rank(M) < 2:
		return False

	a, b = np.linalg.inv(M).dot(u2 - v2)

	return (0 < a < 1) and (0 < b < 1)







if __name__ == '__main__':
	if len(sys.argv) != 2:
		print("USAGE EXAMPLE:\n\n    python robot_navigation.py robot_data.json\n")
		exit(1)
	
	data_file = sys.argv[1]
	f = open(data_file)
	data = json.load(f)
	f.close()

	find_path(data["start"], data["finish"], data["obstacles"])

