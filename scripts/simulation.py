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

## \todo make it more general with respects to objects and robot

import time, sys, os, argparse, rospy
import agimus_hpp.ros_tools as ros_tools
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped
from dynamic_graph_bridge_msgs.msg import Vector
import tf2_ros
from tf2_msgs.msg import TFMessage
from agimus_hpp.client import HppClient
from agimus_hpp.tools import sotTransRPYToHppPose
import hpp_idl

TalosSlicesReducedToFull = [slice(7, 28, 1), slice(33, 34, 1),
                            slice(35, 42, 1), slice(47, 48, 1),
                            slice(49, 51, 1)]

## Conversion of configuration for Talos robot
#
# The robot model in the SoT and in HPP are different. The SoT uses a reduced
# model with less joints. This function updates the configuration of the
# full model from a configuration coming from the SoT
def convertTalosConfigReducedToFull (qReduced, qFull) :
    index = 6
    for s in TalosSlicesReducedToFull:
        width = (s.stop - s.start) // s.step
        qFull [s] = qReduced [index:index+width]
        index += width

## Publish the pose of an object in tf
#
#  Build class with parent and child frames and call instance
#  with pose given as translation and quaternion (x,y,z,X,Y,Z,W)
#  \code
#  pub = PublishObjectPose ('world', 'object/root_joint')
#  pub (x,y,z,X,Y,Z,W)
#  \endcode
class PublishObjectPose (object):
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

## Create dictionary mapping transition names to id's in hpp_idl graph
def createGraphDict (client):
    id = 0
    res = dict ()
    try:
        # loop until an exception is thrown since there is no accessor to
        # the number of graph components.
        while True:
            n = client.manip ().graph.getName (id)
            res [n] = id
            id += 1
    except hpp_idl.hpp.Error as exc:
        return res

class Simulation (object):
    objectPose = {'box' : [0.000, -0.000, 0.857,
                           0.500, -0.500, -0.500, -0.500],
                   'table' : [0,0,0,0,0,0,1]
                   }
    objects = {'box', 'table'}

    subscriberDict = {
        "sot": {
            "transition_name": [String, "computeObjectPositions" ],
            "state" : [Vector, "getRobotState" ],
            },
        }
    def computeRanksInConfiguration (self):
        # Compute ranks of joints in HPP configuration vector
        self.rankInConfiguration = dict ()
        joints = self.client.hpp ().robot.getAllJointNames ()
        rank = 0
        for j in joints:
            size = self.client.hpp().robot.getJointConfigSize (j)
            if size != 0:
                self.rankInConfiguration [j] = rank
                rank += size

    def __init__ (self):
        from threading import Lock
        # Client to HPP
        self.client = HppClient (context = "simulation")
        self.mutex = Lock()
        # Initialize configuration of robot and objects from HPP
        self.q = self.client.hpp ().robot.getConfigSize () * [0.,]
        self.configSize = len (self.q)
        # Compute ranks of joints in HPP configuration
        self.computeRanksInConfiguration ()
        # Initialize ROS node
        rospy.init_node ("simulation")
        rospy.loginfo ("started simulation node")
        self.transitionName = ""
        self.q_rhs = None
        self.objectPublisher = dict ()
        # Create an object publisher by object
        for o in self.objects:
            self.objectPublisher [o] = PublishObjectPose \
                                       ('world', o+'/base_link')
            self.objectPublisher [o].broadcast (self.objectPose [o])

            pose = self.objectPose [o]
            r = self.rankInConfiguration [o + '/root_joint']
            self.q [r:r+7] = pose
        # Create subscribers
        self.subscribers = ros_tools.createSubscribers (self, "/agimus",
                                                        self.subscriberDict)

        # Create mapping from transition names to graph component id in HPP
        self.graphDict = createGraphDict (self.client)

    def getRobotState (self, msg) :
        self.mutex.acquire()
        try:
            # Convert RPY to quaternion
            self.q [:7] = sotTransRPYToHppPose (msg.data [:6])
            convertTalosConfigReducedToFull (msg.data, self.q)
            # update poses of objects
            rospy.loginfo(str(self.q))
            for o in self.objects:
                pose = self.objectPose [o]
                self.objectPublisher [o].broadcast (pose)
                rospy.loginfo(o + " " + str(pose))
        except Exception as e:
            rospy.logerr(e)
        finally:
            self.mutex.release()

    def computeObjectPositions (self, msg) :
        if msg.data == "" : return
        self.mutex.acquire()
        try:
            transitionChanged = msg.data != self.transitionName
            self.transitionName = msg.data
            # if transition changed, record configuration for transition constraint
            # right hand side
            if transitionChanged:
                self.q_rhs = self.q
                self.transitionId = self.graphDict [self.transitionName]
                print ("new transition: " + self.transitionName)
                print ("q_rhs = " + str (self.q_rhs))
            elif self.q_rhs:
                # if transition has not changed and right hand side has been stored,
                # apply transition constraints
                res, self.q, err = \
                  self.client.manip ().graph.applyEdgeLeafConstraints \
                  (self.transitionId, self.q_rhs, self.q)
                self.q = list (self.q)
                for o in self.objects:
                    r = self.rankInConfiguration [o + '/root_joint']
                    self.objectPose [o] = self.q [r:r + 7]
        except Exception as e:
            rospy.logerr(e)
        finally:
            self.mutex.release()

## Create a simulation node that computes the pose of objects from the
#  robot configuration in the Stack of Tasks.

# Create a client to Hpp and instantiate the robot and constraint graph

s = Simulation ()
rospy.spin ()
