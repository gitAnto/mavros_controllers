#!/usr/bin/env python
# coding=utf-8
#*********************************************************************
#*  MIT License
#*
#*  Copyright (c) 2016 Antonio Petitti
#*
#*  Permission is hereby granted, free of charge, to any person obtaining a copy
#*  of this software and associated documentation files (the "Software"), to deal
#*  in the Software without restriction, including without limitation the rights
#*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#*  copies of the Software, and to permit persons to whom the Software is
#*  furnished to do so, subject to the following conditions:
#*  
#*  The above copyright notice and this permission notice shall be included in all
#*  copies or substantial portions of the Software.
#*  
#*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#*  SOFTWARE.
#*********************************************************************

PKG = 'mavros_controllers'
NAME = 'waypoint_navigation'

import rospy
import time
import csv

from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker 

import rospkg

from collections import namedtuple

from controllers.control_position_pid import ControlPositionPid
from controllers.control_position_i_pd import ControlPositionIPD
from controllers.control_position_px4 import ControlPositionPX4
from controllers.control_position_pid_w_dist import ControlPositionPidWithDistance

#from controllers.control_position_cr import ControlPositionCR

def read_wp_from_csv():

	# get an instance of RosPack with the default search paths
	rospack = rospkg.RosPack()

	filename = rospy.get_param('filename', rospack.get_path(PKG) + '/cfg/waypoints.csv')
	#rospy.loginfo("loading waypoints from %s",filename)
	print "loading waypoints from " + filename

	wps = namedtuple('Waypoint', 'id x, y, z, yaw, time')

	reader = csv.reader(open(filename, "rb"))
	next(reader) #parse the first line

	waypoints = []

	for wp in map(wps._make, reader):
		print 'waypoint --> x:' + wp.x + '\ty:' + wp.y + '\tz:' + wp.z + '\tyaw:' + wp.yaw + '\ttime:' + wp.time
		waypoints.append(wp)

	return waypoints;

def choose_pos_controller(pos_controller_type='PX4', tolerance=0.05, rate=10):

	print 'controller ' + pos_controller_type + ' chosen..'

	if pos_controller_type == 'PX4':
		controller = ControlPositionPX4(tolerance, rate)
		
	elif pos_controller_type == 'PID':
		XY_P = rospy.get_param('XY_P',1.25)
		XY_I = rospy.get_param('XY_I', 0.0)
		XY_D = rospy.get_param('XY_D', 0.1)
		Z_P  = rospy.get_param('Z_P', 1.25)
		Z_I  = rospy.get_param('Z_I', 0.001)
		Z_D  = rospy.get_param('Z_D', 0.1)
		controller = ControlPositionPid(XY_P, XY_I, XY_D, Z_P, Z_I, Z_D, tolerance, rate)

	elif pos_controller_type == 'PIDwD':
		XY_P = rospy.get_param('XY_P',1.25)
		XY_I = rospy.get_param('XY_I', 0.0)
		XY_D = rospy.get_param('XY_D', 0.1)
		Z_P  = rospy.get_param('Z_P', 1.25)
		Z_I  = rospy.get_param('Z_I', 0.001)
		Z_D  = rospy.get_param('Z_D', 0.1)
		controller = ControlPositionPidWithDistance(XY_P, XY_I, XY_D, Z_P, Z_I, Z_D, tolerance, rate)

	elif pos_controller_type == 'I-PD':
		XY_P = rospy.get_param('XY_P',1.25)
		XY_I = rospy.get_param('XY_I', 0.0)
		XY_D = rospy.get_param('XY_D', 0.1)
		Z_P  = rospy.get_param('Z_P', 1.25)
		Z_I  = rospy.get_param('Z_I', 0.001)
		Z_D  = rospy.get_param('Z_D', 0.1)
		controller = ControlPositionIPD(XY_P, XY_I, XY_D, Z_P, Z_I, Z_D, tolerance, rate)
		
	else:
		rospy.logerr('controller ' + pos_controller_type + ' not found')
		raise NameError('controller ' + pos_controller_type + ' not found')
		#rospy.signal_shutdown('controller ' + pos_controller_type + ' not found')
		

	print 'controller ' + pos_controller_type + ' loaded..'

	return controller;

def main_loop():

	rospy.init_node(NAME)

	tolerance  = rospy.get_param('tolerance', 0.1)
	rate        = rospy.get_param('rate', 10)

	#pos_controller_type = rospy.get_param('pos_controller_type', 'PX4')

	wayps              = read_wp_from_csv()
	#controller         = ControlPositionPidWithDistance(1.25, 0.0, 0.1, 2.0, 0.0, 0.2, tolerance, rate)#choose_pos_controller(pos_controller_type, tolerance, rate)
	#takeoff_controller = ControlPositionPid(1.25, 0.0, 0.1, 2.0, 0.0, 0.2, 0.3, rate)

	if(safety_check.isSafe()):

		#take-off	
		takeoff_controller = ControlPositionPid(1.25, 0.0, 0.1, 1.0, 0.0, 0.0, 0.3, rate)
		time.sleep(0.5)
		
		print 'take-off waypoint --> x:' + str(takeoff_controller.get_x()) + '\ty:' + str(takeoff_controller.get_y())
		takeoff_controller.set(takeoff_controller.get_x(), takeoff_controller.get_y(), 1.0, 0.2)

		takeoff_controller.done = True

		#waypoints
		with ControlPositionPidWithDistance(1.25, 0.0, 0.1, 2.0, 0.0, 0.2, tolerance, rate) as controller:
		#with ControlPositionCR('PIDwD', tolerance, rate, 1.25, 0.0, 0.1, 2.0, 0.0, 0.2) as controller:
			for wp in wayps:
				print 'UAV moving to WP: ' + wp.id
				controller.set(float(wp.x), float(wp.y), float(wp.z), float(wp.time)) 

		controller.done = True

		# Simulate a slow landing.
		controller.set(0.0, 0.0,  0.8, 0.02)
		controller.set(0.0, 0.0,  0.3, 0.02)
		controller.set(0.0, 0.0,  0.2, 0.02)
		controller.set(0.0, 0.0,  0.1, 0.02)
		controller.set(0.0, 0.0,  0.0, 0.02)
		controller.set(0.0, 0.0, -0.1, 0.02)

	rospy.loginfo("Bye!")


if __name__ == '__main__':
	try:
		main_loop()
	except rospy.ROSInterruptException:
		pass
