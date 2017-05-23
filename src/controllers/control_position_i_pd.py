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
#*  Author: Roberto Colella and Antonio Petitti on Mar, 2016
#*********************************************************************/
 
from tools.ipd import IPD
from controllers.control_position import ControlPosition

#import thread
#import threading
import thread
import threading
import time

#from math import *
import rospy

from mavros import setpoint as SP
from tf.transformations import euler_from_quaternion
 
 
class ControlPositionIPD(ControlPosition):
    """
    This class sends velocity targets to FCU's position controller
    """
    def __init__(self, XY_P, XY_I, XY_D, Z_P, Z_I, Z_D, tollerance=0.05, rate=10, sample_time=0.1, topic='/mavros/local_position/pose', name='pos_controller'):

        ControlPosition.__init__(self, tollerance, rate, topic, name) 

        self.sample_time = sample_time

        # publisher for mavros/setpoint_velocity/cmd_vel
        self.pub = SP.get_pub_velocity_cmd_vel(queue_size=10)

        self.ipdx = IPD(XY_P, XY_I, XY_D)
        self.ipdy = IPD(XY_P, XY_I, XY_D)
        self.ipdz = IPD(Z_P, Z_I, Z_D)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        try:
            thread.start_new_thread(self.control_loop, ())
        except:
            fault("Error: Unable to start thread")

        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()

        
 
    def control_loop(self):

        self.ipdx.setSampleTime(self.sample_time)
        self.ipdy.setSampleTime(self.sample_time)
        self.ipdz.setSampleTime(self.sample_time)

        rate = rospy.Rate(self.rate)   # 10hz
 
        msg = SP.TwistStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )
 
        while not rospy.is_shutdown():
            self.ipdx.update(self.x)
            self.ipdy.update(self.y)
            self.ipdz.update(self.z)
            
            msg.twist.linear.x = self.ipdx.output
            msg.twist.linear.y = self.ipdy.output
            msg.twist.linear.z = self.ipdz.output

            self.pub.publish(msg)
            rate.sleep()
 

    def set(self, x, y, z, delay=0.0, wait=True):

        self.done = False
        self.x_ref = x 
        self.y_ref = y
        self.z_ref = z 

        self.ipdx.SetPoint=self.x_ref
        self.ipdy.SetPoint=self.y_ref    
        self.ipdz.SetPoint=self.z_ref     
 
        if wait:
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()
 
        time.sleep(delay)
 
 
    def update_gains(self, P_Z, I_Z, D_Z, P_XY, I_XY, D_XY):
     
        self.ipdz.setKp(P_Z)
        self.ipdz.setKi(I_Z)
        self.ipdz.setKd(D_Z)
        self.ipdx.setKp(P_XY)
        self.ipdx.setKi(I_XY)
        self.ipdx.setKd(D_XY)
        self.ipdy.setKp(P_XY)
        self.ipdy.setKi(I_XY)
        self.ipdy.setKd(D_XY)
