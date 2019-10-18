#!/usr/bin/python

# Copyright 2019 CNRS - Airbus SAS
# Authors: Florent Lamiraux, Joseph Mirabel, Alexis Nicolin
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import time, sys, os, argparse, rospy
import agimus_hpp.ros_tools as ros_tools
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_msgs.msg import TFMessage
from agimus_hpp.client import HppClient

## Publish the pose of an object in tf2
#
#  Build class with parent and child frames and call instance
#  with pose given as translation and quaternion (x,y,z,X,Y,Z,W)
#  \code
#  pub = publishObjectPose ('world', 'object/root_joint')
#  pub (x,y,z,X,Y,Z,W)
#  \endcode
class publishObjectPose (object):
    ## Constructor
    #  \param parent name of parent frame,
    #  \param child name of child frame
    def __init__ (self, parent, child):
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.ts = TransformStamped()
        self.ts.header.frame_id = parent
        self.ts.child_frame_id = child

    ## publish pose between parent and child frames
    #  \param q pose as a 7 dimensional vector (x,y,z,X,Y,Z,W)
    def broadcast (self, q):
        self.ts.header.stamp = rospy.Time.now()
        self.ts.transform.translation.x = q[0]
        self.ts.transform.translation.y = q[1]
        self.ts.transform.translation.z = q[2]
        self.ts.transform.rotation.x = q[3]
        self.ts.transform.rotation.y = q[4]
        self.ts.transform.rotation.z = q[5]
        self.ts.transform.rotation.w = q[6]
        res = self.broadcaster.sendTransform (self.ts)


class Simulation (object):
    boxPose = [0.45891797741593393, -0.15, 0.8374964472840138,
               -0.5, 0.5, 0.5, 0.5]
    subscriberDict = {
        "sot": {
            "transition_name": [String, "computeObjectPositions" ],
            },
        }
    def __init__ (self):
        rospy.init_node ("simulation")
        rospy.loginfo ("started simulation node")
        self.boxPosePublisher = publishObjectPose ('world', 'box/base_link')
        self.subscribers = ros_tools.createSubscribers (self, "/agimus",
                                                        self.subscriberDict)
    def spin (self):
        self.boxPosePublisher.broadcast (self.boxPose)

    def computeObjectPositions (self, msg) :
        self.transitionName = msg.data

## Create a simulation node that computes the pose of objects from the
#  robot configuration in the Stack of Tasks.

# Create a client to Hpp and instantiate the robot and constraint graph

s = Simulation ()
while not rospy.is_shutdown ():
    s.spin ()
    time.sleep (.1)

