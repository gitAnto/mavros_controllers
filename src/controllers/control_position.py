#!/usr/bin/env python
#*********************************************************************
#*
#*  Copyright (c) 2014-2016
#*  All rights reserved.
#*
#*  Software License Agreement (BSD License)
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     the distribution.
#*   * Neither the name of the author nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#*
#*  Author: Antonio Petitti on Mar, 2016
#*********************************************************************/


import rospy
import mavros

import thread
import threading

from math import *

from mavros import setpoint as SP
from geometry_msgs.msg import TransformStamped
from mavros import command

import os
import xmlrpclib


class ControlPosition:
	"""
	This class sends velocity targets to FCU's position controller
	"""
	def __init__(self, tolerance=0.05, rate=10, topic='/mavros/local_position/pose', name='pos_controller'):
		
		self.tolerance = tolerance #meters
		self.rate = rate  #rate 

		self.name = name #name

		self.wp_reached = True
		self.task_done  = True

		self.done_evt = threading.Event()

		self.x_ref = 0.0
		self.y_ref = 0.0
		self.z_ref = 0.0
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		self.pose_top = 0
		self.transf_top = 0

		self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])

		code, statusMessage, topicTypes = self.m.getTopicTypes(self.name)

		#print topicTypes

		for topic_pair in topicTypes:
			if topic_pair[0] == topic:
				if topic_pair[1] == 'geometry_msgs/TransformStamped':
					self.transf_top = 1
					break

				if topic_pair[1] == 'geometry_msgs/PoseStamped':
					self.pose_top = 1
					break   

		if not self.pose_top and not self.transf_top:
			rospy.logerr("ERROR: topic %s has a wrong data type", topic)

		rospy.loginfo("Controller input on topic: %s", topic)
		rospy.loginfo("transf_top: %d", self.transf_top)
		rospy.loginfo("pose_top: %d", self.pose_top)
		
		# publisher for mavros/setpoint_velocity/cmd_vel
		self.pub = None #empty publisher to be defined in the child class

		mavros.set_namespace()

		# subscriber for mavros/local_position/local
		if self.pose_top:
			self.sub = rospy.Subscriber(topic,#topic must be a pose topic!
					SP.PoseStamped, self.local_pos_callback) 
		else:
			self.sub = rospy.Subscriber(topic,#topic must be a pose topic!
					TransformStamped, self.local_pos_callback) 
 
	def __enter__(self):
		return self
 
	def __exit__(self, *err):
		return True

	def control_loop(self):

		print "virtual method"

		"""
		This method sends reference targets to FCU
		"""
 
	def _arm(self, state):
		try:
			ret = command.arming(value=state)
		except rospy.ServiceException as ex:
			fault(ex)
		if not ret.success:
			fault("Request failed. Check mavros logs")

		return ret		
 
	def set(self):
		
		print "virtual method"

		"""
		This method sets the position reference
		"""
 
	def local_pos_callback(self, topic):
 
		if self.pose_top:
			#print 'I am in the callback!'
			self.x = topic.pose.position.x
			self.y = topic.pose.position.y
			self.z = topic.pose.position.z

		else:
			#print 'I am in the callback!'
			self.x = topic.transform.translation.x
			self.y = topic.transform.translation.y
			self.z = topic.transform.translation.z

		rospy.loginfo("Following values read -- x: %f  y: %f  z: %f", self.x, self.y, self.z)
		rospy.loginfo("Errors: %f  %f  %f", self.x - self.x_ref, self.y - self.y_ref, self.z - self.z_ref)

		if self.is_near('X', self.x, self.x_ref, self.tolerance) and \
		   self.is_near('Y', self.y, self.y_ref, self.tolerance) and \
		   self.is_near('Z', self.z, self.z_ref, self.tolerance) :
			self.wp_reached = True
			self.done_evt.set() 


	def is_near(self, msg, x, y, tolerance):
		rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
						   msg, x, y, abs(x - y))
		#print("Position %s: local: %d, target: %d, abs diff: %d",
		#				   msg, x, y, abs(x - y))
		return abs(x - y) < tolerance


	def get_x(self):
		return self.x

	def get_y(self):
		return self.y

	def get_z(self):
		return self.z

	def is_reached(self):
		return self.wp_reached

	def is_done(self):
		return self.task_done
