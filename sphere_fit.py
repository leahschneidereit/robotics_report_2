#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

xyz = XYZarray()
sphere_params = SphereParams()
computed = False

def solver(data):

	global computed
	
	points = data.points
	
	# develop A and B matrices
	A = []
	B = []
	
	# iterate over points and add to matrices for xyz
	for i in range (len(points)):
		A2 = []
		A2.append(2*points[i].x)
		A2.append(2*points[i].y)
		A2.append(2*points[i].z)
		A2.append(1)
		A.append(A2)
		B.append(points[i].x**2 + points[i].y**2 + points[i].z**2)
	
	# calculate P
	P = np.linalg.lstsq(A,B,rcond = None)[0]
	
	# calculate radius
	r = np.sqrt(P[0]**2 + P[1]**2 + P[2]**2 + P[3])
	
	
	# assign values to sphere data 
	sphere_params.xc = P[0]
	sphere_params.yc = P[1]
	sphere_params.zc = P[2]
	sphere_params.radius = r
	
	# set flag to True
	computed = True
	
def filter(data, fil_out, fil_gain, first_round):
	if first_round:
		fil_out = data
	else:
		# fil_in for point is input value
		fil_in = data
		# fil_out calculated using gain and fil_in 
		fil_out = fil_gain * fil_in + (1- fil_gain) * fil_out
	
	return fil_out
	
if __name__ == '__main__':
	# define node
	rospy.init_node('sphere_fit', anonymous = True)
	# define a subscriber to read images
	img_sub = rospy.Subscriber('/xyz_cropped_ball', XYZarray, solver) 
	# define a publisher to publish images
	img_pub = rospy.Publisher('sphere_params', SphereParams, queue_size = 1)
	
	# set the loop frequency
	rate = rospy.Rate(10)
	
	# set filters 
	fil_out_x = sphere_params.xc
	fil_out_y = sphere_params.yc
	fil_out_z = sphere_params.zc
	fil_out_radius = sphere_params.radius
	
	# set gains
	point_gain = 0.05
	radius_gain = 0.1
	
	first_round = True
	while not rospy.is_shutdown():
		print('main loop')
		# publish if data has been calculated 
		if computed:
			print('BEFORE filter\n')
			# call filter function for each point and radius to calculate fil_out
			print('fil_out_x:', fil_out_x)
			fil_out_x = filter(sphere_params.xc, fil_out_x, point_gain, first_round)
			print('fil_out_x:', fil_out_x)
			fil_out_y = filter(sphere_params.yc, fil_out_y,point_gain, first_round)
			fil_out_z = filter(sphere_params.zc, fil_out_z,point_gain, first_round)
			fil_out_radius = filter(sphere_params.radius, fil_out_radius, radius_gain,first_round)
			print('AFTER filter\n')
			first_round = False
			
			#filter xc
			sphere_params.xc = fil_out_x
			sphere_params.yc = fil_out_y
			sphere_params.zc = fil_out_z
			sphere_params.radius = fil_out_radius
			
			#publish sphere_params
			img_pub.publish(sphere_params)
			computed = False 
		#pause until the next iteration			
		rate.sleep()
