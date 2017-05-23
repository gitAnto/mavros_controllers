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
 
from controllers.control_position import ControlPosition

import thread
import threading
import time

import rospy
 
from math import *

from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
 
 
class ControlPositionPX4(ControlPosition):
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self, tollerance=0.05, rate=10, topic='/mavros/local_position/pose', name='pos_controller'):

        ControlPosition.__init__(self, tollerance, rate, topic, name)

        # publisher for mavros/setpoint_position/local
        self.pub = SP.get_pub_position_local(queue_size=10)

        try:
            thread.start_new_thread(self.control_loop, ())
        except:
            fault("Error: Unable to start thread")

        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()

 
    def control_loop(self):
        rate = rospy.Rate(self.rate)   # 10hz
 
        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )
 
        while not rospy.is_shutdown():
            msg.pose.position.x = self.x_ref
            msg.pose.position.y = self.y_ref
            msg.pose.position.z = self.z_ref
 
            # For demo purposes we will lock yaw/heading to north.
            yaw_degrees = 0  # North
            yaw = radians(yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            msg.pose.orientation = SP.Quaternion(*quaternion)
 
            self.pub.publish(msg)
            rate.sleep()
 
    def set(self, x, y, z, delay=0.0, wait=True):
        self.done = False
        self.x_ref = x
        self.y_ref = y
        self.z_ref = z
 
        if wait:
            rate = rospy.Rate(5)
            t_init = rospy.Time.now()
            while not self.done and not rospy.is_shutdown():
                elapsed_time = rospy.Time.now() - t_init

                if elapsed_time >= delay: 
                    self.done = True

		rate.sleep()
 
        #time.sleep(delay)

