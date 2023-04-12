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
received_params = False
sphere_x = 0
sphere_y = 0
sphere_z = 0
radius = 0

def get_sphere_params(data):
	global sphere_params
	global received_params
	global sphere_x
	global sphere_y
	global sphere_z
	
	sphere_x = data.xc
	sphere_y = data.yc
	sphere_z = data.zc
	sphere_radius = data.radius
	
	received_params = True

if __name__ == '__main__':
	# initialize node
	rospy.init_node('simple_planner', anonymous = True)
	# subsriber to get sphere params
	sphere_sub = rospy.Subscriber('/sphere_params', SphereParams, get_sphere_params)
	# add publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	# add listener 
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
		
	# intitialize Quaternion message
	q_rot = Quaternion()
	while not rospy.is_shutdown():
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
	
	
		# define a plan variable
		plan = Plan()
		plan_point1 = Twist()
	
		# define a point close to the initial (initialization) position
		plan_point1.linear.x = -0.792
		plan_point1.linear.y = -0.134
		plan_point1.linear.z = 0.363
		plan_point1.angular.x = 3.14
		plan_point1.angular.y = 0
		plan_point1.angular.z = 1.57
		# add this point to the plan
		plan.points.append(plan_point1)
	
		plan_point2 = Twist()
	
		# define point 2 
		plan_point2.linear.x = x
		plan_point2.linear.y = y
		plan_point2.linear.z = z
		plan_point2.angular.x = 3.14
		plan_point2.angular.y = 0
		plan_point2.angular.z = 1.57
		# add this point to the plan
		plan.points.append(plan_point2)
	
		plan_point3 = Twist()
		# define point 3 - diagonal upward motion
		plan_point3.linear.x = -0.792
		plan_point3.linear.y = 0.15
		plan_point3.linear.z = 0.363
		plan_point3.angular.x = 3.14
		plan_point3.angular.y = 0
		plan_point3.angular.z = 1.57
		# add this point to the plan
		plan.points.append(plan_point3)
	
	
		plan_point4 = Twist()
		# define point 4 - downward motion 
		plan_point4.linear.x = -0.792
		plan_point4.linear.y = 0.15
		plan_point4.linear.z = 0.15
		plan_point4.angular.x = 3.14
		plan_point4.angular.y = 0
		plan_point4.angular.z = 1.57
		# add this point to the plan
		plan.points.append(plan_point4)
	
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
