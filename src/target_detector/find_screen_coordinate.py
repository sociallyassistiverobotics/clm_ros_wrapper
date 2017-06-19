"""
this script is used to transform the coordinates in the screen frame to the world frame
"""

"""
the origin in the screen frame is the bottom left corner
"""
def origin():
	return (100, 100, 110)

"""
add two 3-D cooredinates in the world frame
"""
def add(coordinate_1, coordinate_2):
	return (coordinate_1[0] + coordinate_2[0], coordinate_1[1] + coordinate_2[1], coordinate_1[2] + coordinate_2[2])

"""
The corresponding change in the world frame when there is a change of length length
in the x direction in the screen frame. The x coordinate is the long side
"""
def length_change_x(length):
	return (length * 310 / 352, length * 167 / 352, 0)

"""
The corresponding change in the world frame when there is a change of length length
in the y direction in the screen frame. The x coordinate is the short side
"""
def length_change_y(length):
	return (-length * 70 / 317, length * 180 / 317, length * 280 / 317)

"""
The corresponding change in the 3-D world frame when there is a 2-D change in the screen frame 
"""
def length_change(coordinate):
	result = (0, 0, 0)
	result = add(length_change_x(coordinate[0]), result)
	result = add(length_change_y(coordinate[1]), result)
	return result

if __name__ == '__main__':
	for x in [0, 120, 240, 360]:
		for y in [160, 80, 0]:
			print "(" + str(x) + ", " + str(y) + "): " + str(add(length_change((x, y)), origin()))
