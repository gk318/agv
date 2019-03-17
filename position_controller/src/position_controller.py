#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Tobias Mueller. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the association nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This script is used to control the velocity with position setpoints.
"""
# Python imports
import argparse
import time
import numpy
from math import sin, cos, atan2, hypot, fabs, sqrt, pi

# ROS imports
import rospy
import tf
import tf2_ros

# Messages
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Bool

# Services
from std_srvs.srv import *


class PositionController(object):


    def __init__(self, name):
        '''
        @brief 
        '''

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #self.service = rospy.Service('/agv_mecanum/enable_pos_ctrl', SetBool, self.handle_service)
        self.activator_x = rospy.Publisher("/agv_mecanum/pid_x/pid_enable", Bool, queue_size=1)
        self.activator_y = rospy.Publisher("/agv_mecanum/pid_y/pid_enable", Bool, queue_size=1)
        self.activator_yaw = rospy.Publisher("/agv_mecanum/pid_yaw/pid_enable", Bool, queue_size=1)
        self.sub_pid_x_effort = rospy.Subscriber("/agv_mecanum/pid_x/control_effort", Float64, self.callback_x)
        self.sub_pid_y_effort = rospy.Subscriber("/agv_mecanum/pid_y/control_effort", Float64, self.callback_y)
        self.sub_pid_yaw_effort = rospy.Subscriber("/agv_mecanum/pid_yaw/control_effort", Float64, self.callback_yaw)
        self.pub_pid_x_state = rospy.Publisher("/agv_mecanum/pid_x/state", Float64, queue_size=1)
        self.pub_pid_y_state = rospy.Publisher("/agv_mecanum/pid_y/state", Float64, queue_size=1)
        self.pub_pid_yaw_state = rospy.Publisher("/agv_mecanum/pid_yaw/state", Float64, queue_size=1)
        self.pub_pid_x_setpoint = rospy.Publisher("/agv_mecanum/pid_x/setpoint", Float64, queue_size=1)
        self.pub_pid_y_setpoint = rospy.Publisher("/agv_mecanum/pid_y/setpoint", Float64, queue_size=1)
        self.pub_pid_yaw_setpoint = rospy.Publisher("/agv_mecanum/pid_yaw/setpoint", Float64, queue_size=1)
        self.sub_sp = rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, self.callbackSetpoint)
        self.pub_cmd = rospy.Publisher("/agv_mechanum/cmd_vel", Twist, queue_size=100)
	#self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=100)

        self.cmd = Twist()
        self.pos_x = .0
        self.pos_y = .0
        self.yaw = .0
        
        self.x_prev = .0
        self.y_prev = .0
        self.yaw_prev = .0
        self.time_prev = .0


        self.sp_pos_x = Float64()
        self.sp_pos_y = Float64()
        self.sp_yaw = Float64()
        self.control_effort_x = Float64()
        self.control_effort_y = Float64()
        self.control_effort_yaw = Float64()

        self.path = Path()
        self.i = 0

        self.t = TransformStamped()
        self.t.header.frame_id = "odom"
        #self.t.header.frame_id = "agv_base_footprint"
        self.t.child_frame_id = "setpoint_pose"
        self.t.transform.translation.x = 0
        self.t.transform.translation.y = 0
        self.t.transform.translation.z = 0
        self.t.transform.rotation.x = 0
        self.t.transform.rotation.y = 0
        self.t.transform.rotation.z = 0
        self.t.transform.rotation.w = 1

        self.br = tf2_ros.TransformBroadcaster()


        self.pid_enabled = True
        self.planner_enabled = False

        rospy.on_shutdown(self.cleanup)

        rospy.loginfo("Start position controller.")

    def callbackSetpoint(self, msg):
        self.path = msg
        self.planner_enabled = True


    def cleanup(self):
        '''
        @brief destructor
        '''
        #self.handle_service(False)
        rospy.loginfo("Stop position controller")

    def setpointTransform(self, msg):
        self.t.transform.translation.x = msg.pose.position.x
        self.t.transform.translation.y = msg.pose.position.y
        self.t.transform.translation.z = msg.pose.position.z
        self.t.transform.rotation.x = msg.pose.orientation.x
        self.t.transform.rotation.y = msg.pose.orientation.y
        self.t.transform.rotation.z = msg.pose.orientation.z
        self.t.transform.rotation.w = msg.pose.orientation.w
        self.t.header.stamp = rospy.Time.now()
        self.br.sendTransform(self.t)



    def lookupTransform(self, parent, child):
        try:
            trans = self.tfBuffer.lookup_transform(parent, child, rospy.Time())
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False


    def atSetpointPos(self):
        #deadband = 0.025
        deadband = 0.001
        disp = False
        disp = self.lookupTransform("agv_base_footprint", "setpoint_pose")
        if disp:
            displacement = sqrt(disp.transform.translation.x**2 + disp.transform.translation.y**2)
            if displacement <= deadband:
                return True
            else:
                return False
        else:
            return False


    def atSetpointYaw(self):
        deadband = 0.17453 # 10 deg in rad
        disp = False
        disp = self.lookupTransform("agv_base_footprint", "setpoint_pose")
        if disp:
            quat = [disp.transform.rotation.x, disp.transform.rotation.y, 
                    disp.transform.rotation.z, disp.transform.rotation.w]
            euler = tf.transformations.euler_from_quaternion(quat)
            yaw = euler[2]
            if yaw <= deadband:
                return True
            else:
                return False
        else:
            return False


    def control(self):
        '''
        @brief 
        '''
        if self.pid_enabled and self.planner_enabled:
		if self.i < len(self.path.poses):
			pose = self.path.poses[self.i]
			self.setpointTransform(pose)
			
			setpoint = False
			trans = False
		    
		        setpoint = self.lookupTransform("agv_base_footprint", "setpoint_pose")
		        trans = self.lookupTransform("setpoint_pose", "agv_base_footprint")

		    
		        if setpoint and trans:
			    #setpoint
			
			    self.pub_pid_x_setpoint.publish(0)
			    self.pub_pid_y_setpoint.publish(0)
			    self.pub_pid_yaw_setpoint.publish(0)

			    #trans
			
			    self.pos_x = trans.transform.translation.x
			    self.pos_y = trans.transform.translation.y
			    quat = [trans.transform.rotation.x, trans.transform.rotation.y, 
				    trans.transform.rotation.z, trans.transform.rotation.w]
			    euler = tf.transformations.euler_from_quaternion(quat)
			    self.yaw = euler[2]
			    self.pub_pid_x_state.publish(self.pos_x)
			    self.pub_pid_y_state.publish(self.pos_y)
			    self.pub_pid_yaw_state.publish(self.yaw)

			    if not self.atSetpointYaw():
			        self.cmd.linear.x = 0
			        self.cmd.linear.y = 0
			        self.cmd.angular.z = self.control_effort_yaw.data 
			        self.pub_cmd.publish(self.cmd)

			    elif not self.atSetpointPos():
			        self.cmd.linear.x = self.control_effort_x.data
			        self.cmd.linear.y = self.control_effort_y.data
			        self.cmd.angular.z = self.control_effort_yaw.data
			        self.pub_cmd.publish(self.cmd)

			    else:
			        self.i += 1

		else:
			self.cmd.linear.x = 0
		        self.cmd.linear.y = 0
		        self.cmd.angular.z = 0
			self.planner_enabled = False
		        #self.pid_enabled = False 
		        self.pub_cmd.publish(self.cmd)

    def callback_x(self, msg):
        self.control_effort_x = msg

    def callback_y(self, msg):
        self.control_effort_y = msg

    def callback_yaw(self, msg):
        self.control_effort_yaw = msg

    #def handle_service(self, req):
     #   rate = rospy.Rate(25)
        #check if flag is toggled
      #  if req.data != self.pid_enabled:
            # toggle PID controller to prevent integration
            # dirtyfix: for loop secures connection between sub and pub
       #     for i in range(1, 5):
        #        self.activator_x.publish(req.data)
         #       self.activator_y.publish(req.data)
          #      self.activator_yaw.publish(req.data)
           #     rate.sleep()
            # set controll effort equal zero
            #self.cmd.linear.x = 0
            #self.cmd.linear.y = 0
            #self.cmd.angular.z = 0
            #self.pub_cmd.publish(self.cmd)
            #self.pid_enabled = req.data
            #return SetBoolResponse(True, "Positioncontroller set to {}".format(req.data))
        #return SetBoolResponse(True, "Positioncontroller was set to {}".format(req.data))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = __doc__)
    # Accept control rate. Default is 50 hz.
    parser.add_argument("--rate", help="Controller rate", type=float, default=50.0)
    args, unknown = parser.parse_known_args()

    try:
        rospy.init_node("position_controller")

        pc = PositionController(rospy.get_name())
        rate = rospy.Rate(args.rate)

        # Keep looping unless the system receives shutdown signal.
        while not rospy.is_shutdown():
            pc.control()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    rospy.spin()
