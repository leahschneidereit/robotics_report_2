#!/usr/bin/env python3

import rospy
import math

#import messages
import tf2_ros
from tf.transformations import *

#import plan and twist message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams
from geometry_msgs.msg import Quaternion
from tf2_geometry_msgs import PointStamped



sphere_params = SphereParams()
sphere_x = 0
sphere_y = 0
sphere_z = 0
radius = 0
current_pos = [0] * 6
initialized = False
received_params = False

def get_sphere_params(data):
	global sphere_params
	global received_params
	global sphere_x
	global sphere_y
	global sphere_z
	
	received_params = True
	
	sphere_x = data.xc
	sphere_y = data.yc
	sphere_z = data.zc
	sphere_radius = data.radius
	print(received_params)


def get_position(data):
	global current_pos
	global initialized 
	
	current_pos[0] = data.linear.x
	current_pos[1] = data.linear.y
	current_pos[2] = data.linear.z
	current_pos[3] = data.angular.x
	current_pos[4] = data.angular.y
	current_pos[5] = data.angular.z
	
	initialized = True
	
def make_plan(linx, liny, linz, angx, angy, angz):
	plan_point = Twist()
	
	plan_point.linear.x = linx
	plan_point.linear.y = liny
	plan_point.linear.z = linz
	plan_point.angular.x = angx
	plan_point.angular.y = angy
	plan_point.angular.z = angz
	
	return plan.points.append(plan_point)

	
if __name__ == '__main__':
	# initialize node
	rospy.init_node('simple_planner', anonymous = True)
	# subsriber to get sphere params
	sphere_sub = rospy.Subscriber('/sphere_params', SphereParams, get_sphere_params)
	# add publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# add subscriber for receiving information from driver
	loc_sub = rospy.Subscriber('/ur5e/toolpose', Twist, get_position)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	# add listener 
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
		
	# intitialize Quaternion message
	q_rot = Quaternion()
	planned = False
	while not rospy.is_shutdown():
		print(planned, initialized, received_params)
		if initialized and received_params and not planned:
			try:
				trans = tfBuffer.lookup_transform("base","camera_color_optical_frame", rospy.Time())
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print('Frames not available')
				loop_rate.sleep()
				continue
	
			# define target point for tooltip 
			pt_in_camera = PointStamped()
			pt_in_camera.header.frame_id = 'camera_color_optical_frame'
			pt_in_camera.header.stamp = rospy.get_rostime()
			
			pt_in_camera.point.x = sphere_x
			pt_in_camera.point.y = sphere_y
			pt_in_camera.point.z = sphere_z
		
			#convert to base frame coordinates
			pt_in_base = tfBuffer.transform(pt_in_camera, 'base', rospy.Duration(1.0))
			x = pt_in_base.point.x
			y = pt_in_base.point.y
			z = pt_in_base.point.z
			r = sphere_params.radius
		
			# define roll, pitch, yaw values
			roll = 3.14 
			pitch = 0
			yaw = 1.57
			z_offset = .05
			# define a plan variable
			plan = Plan()
			plan_point1 = Twist()
			
			# add points to plan
			make_plan(current_pos[0],current_pos[1], current_pos[2], roll, pitch, yaw)
			make_plan(x,y,z+.3, roll, pitch, yaw)
			make_plan(x,y,z + z_offset, roll, pitch, yaw)
			make_plan(-0.792, 0.15, 0.363, roll, pitch, yaw)
			make_plan(-0.792, 0.15, 0.15,roll, pitch, yaw)
			
			planned = True
		
			# publish the plan
			plan_pub.publish(plan)
			
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
