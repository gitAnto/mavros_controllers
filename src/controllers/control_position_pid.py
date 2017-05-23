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
#*  Author: Roberto Colella, Antonio Petitti, and Donato Di Paola on Mar, 2016
#*********************************************************************/
 
from tools.pid import PID
from controllers.control_position import ControlPosition

import thread
#import threading
import time

#from math import *
import rospy

from mavros import setpoint as SP
from tf.transformations import euler_from_quaternion
 
 
class ControlPositionPid(ControlPosition):
	"""
	This class sends velocity targets to FCU's position controller
	"""
	def __init__(self, XY_P, XY_I, XY_D, Z_P, Z_I, Z_D, tolerance=0.05, rate=10, sample_time=0.1, topic='/mavros/local_position/pose', name='pos_controller'):

		ControlPosition.__init__(self, tolerance, rate, topic, name) 

		self.sample_time = sample_time

		# publisher for mavros/setpoint_velocity/cmd_vel
		self.pub = SP.get_pub_velocity_cmd_vel(queue_size=10)

		self.pidx = PID(XY_P, XY_I, XY_D)
		self.pidy = PID(XY_P, XY_I, XY_D)
		self.pidz = PID(Z_P, Z_I, Z_D)

		self.pidx.setSampleTime(self.sample_time)
		self.pidy.setSampleTime(self.sample_time)
		self.pidz.setSampleTime(self.sample_time)

		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		try:
			thread.start_new_thread(self.control_loop, ())
		except:
			fault("Error: Unable to start thread")
		
 
	def control_loop(self):

		rate = rospy.Rate(self.rate)   # 10hz
 
		msg = SP.TwistStamped(
			header=SP.Header(
				frame_id="base_footprint",  # no matter, plugin don't use TF
				stamp=rospy.Time.now()),	# stamp should update
		)
 

		#while not self.task_done:
		while not rospy.is_shutdown():

			if not self.task_done:

				self.pidx.update(self.x)
				self.pidy.update(self.y)
				self.pidz.update(self.z)

				msg.twist.linear.x = self.pidx.output
				msg.twist.linear.y = self.pidy.output
				msg.twist.linear.z = self.pidz.output

				#print self.name + " is publishing"
				self.pub.publish(msg)
				rate.sleep()
			
 

	def set(self, x, y, z, delay=0.0):

		self.task_done  = False
		self.wp_reached = False
		self.x_ref = x 
		self.y_ref = y
		self.z_ref = z 

		self.pidx.SetPoint=self.x_ref
		self.pidy.SetPoint=self.y_ref	
		self.pidz.SetPoint=self.z_ref	 

		rate = rospy.Rate(self.rate)


				#going to the wp
		while not self.wp_reached and not rospy.is_shutdown():
			rate.sleep()

				#stay in the wp
		wp_duration  = rospy.Duration.from_sec(delay)
		t_init	   = rospy.Time.now()

		#print 'waypoint reached'

		while not self.task_done and not rospy.is_shutdown():

			elapsed_time = rospy.Time.now() - t_init

			if elapsed_time >= wp_duration: 
				self.task_done = True

			rate.sleep()

		#print 'task done'
 
 
	def update_gains(self, P_Z, I_Z, D_Z, P_XY, I_XY, D_XY):
	 
		self.pidz.setKp(P_Z)
		self.pidz.setKi(I_Z)
		self.pidz.setKd(D_Z)
		self.pidx.setKp(P_XY)
		self.pidx.setKi(I_XY)
		self.pidx.setKd(D_XY)
		self.pidy.setKp(P_XY)
		self.pidy.setKi(I_XY)
		self.pidy.setKd(D_XY)
