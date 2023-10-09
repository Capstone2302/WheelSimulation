#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_srvs.srv import Empty
from datetime import datetime
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

class CommandToJointState:

    def __init__(self):
        self.center_pixel = 399
        self.Kp = 0.1
        self.Ki = 0
        self.Kd = 0
        self.prev_err = 0
        self.dt = (datetime.now() - datetime.now()).microseconds
        self.t = datetime.now()
        self.wheel_pos_write = 0.0
        self.wheel_pos_read = 0.0
        self.ball_pos_x = 0
        self.ball_pos_y = 0
        self.integral = 0
        self.prev_err=0
        self.joint_name = 'rev'
        self.joint_state = JointState()
        self.joint_state.name.append(self.joint_name)
        self.joint_state.position.append(0.0)
        self.joint_state.velocity.append(0.0)
        self.bridge = CvBridge()
        self.joint_pub = rospy.Publisher("/wheel/rev_position_controller/command", Float64, queue_size=1)
        self.wheel_sub = message_filters.Subscriber('/wheel/joint_states', JointState)
        self.ball_sub = message_filters.Subscriber("/wheel/camera1/image_raw", Image)
        ats = ApproximateTimeSynchronizer([self.wheel_sub, self.ball_sub], queue_size=5, slop=0.1)
        self.reset_ball_pos()
        ats.registerCallback(self.pootLovato)


    def PID_control(self):
        self.dt = ((datetime.now() - self.t).microseconds)/1000.0
        self.t = datetime.now()
        error = self.ball_pos_x
        self.integral = self.integral + error*self.dt
        derivative = (error - self.prev_err)/self.dt
        self.wheel_pos_write = self.wheel_pos_read + self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        self.prev_err = error
        
        self.joint_pub.publish(self.wheel_pos_write)
        rospy.loginfo('weel pos published: '+ str(self.wheel_pos_write))

    def reset_ball_pos(self):    
        state_msg = ModelState()
        state_msg.model_name = 'ball'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.5
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.loginfo('ball reset')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException:
            print( "Service call failed")

    def get_wheel_pos_callback(self, msg):
        msg_str = str(msg)
        i = msg_str.find('position: [')+11
        self.wheel_pos_read = float(msg_str[i:i+12])
        
        rospy.loginfo('wheel pos read: '+ str(self.wheel_pos_read))
        # self.wheel_pos_write = self.wheel_pos_read+1
        # self.joint_pub.publish(self.wheel_pos_write)
        # rospy.loginfo('position published: '+ str(self.wheel_pos_read+1))

    def get_ball_pos_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        output = cv_image.copy()
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2,20, 
                                   param1=50,
                                   param2=30,
                                   minRadius=0,
                                   maxRadius=12)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                # cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                # rospy.loginfo(str(y))
                self.ball_pos_x = x - self.center_pixel #neg ball_pos means left of centre, pos = right of center
                self.ball_pos_y = y
                rospy.loginfo('ball pos x read: '+ str(self.ball_pos_x))
                rospy.loginfo('ball pos y read: '+ str(self.ball_pos_y))
                if self.ball_pos_y >450:
                    self.reset_ball_pos()
                
                self.PID_control()
        # cv2.imshow("output", np.hstack([cv_image, output]))
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(1)


    def pootLovato(self, jointState,image):
        self.get_wheel_pos_callback(jointState)
        self.get_ball_pos_callback(image)

if __name__ == '__main__':
    rospy.init_node('command_to_joint_state')
    command_to_joint_state = CommandToJointState()
    rospy.spin()
