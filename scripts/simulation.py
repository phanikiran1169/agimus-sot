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

import time, sys, os, argparse, rospy, traceback
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

def initialize():
    # Initialize ROS node
    rospy.init_node ("simulation")
    rospy.loginfo ("started simulation node")

    from hpp.corbaserver import createContext
    if createContext("simulation"):
        # initialize the simulation
        if rospy.has_param("~initialization_command"):
            cmd = rospy.get_param("~initialization_command")
            rospy.loginfo ("Initialization with " + cmd)
            os.system(cmd)
        else:
            msg = "Server is not initialized and ros parameter initialization_command is not set."
            rospy.logerr (msg)
            raise RuntimeError(msg)
    else:
        rospy.loginfo ("Simulation already initialized.")

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
    subscriberDict = {
        "sot": {
            "transition_name": [String, "computeObjectPositions" ],
            "state" : [Vector, "getRobotState" ],
            },
        }
    def computeRanksInConfiguration (self):
        # Compute ranks of joints in HPP configuration vector
        hpp = self.client.hpp()
        self.rankInConfiguration = dict ()
        joints = hpp.robot.getAllJointNames ()
        rank = 0
        for j in joints:
            size = hpp.robot.getJointConfigSize (j)
            if size != 0:
                self.rankInConfiguration [j] = rank
                rank += size

    def initializeObjects (self, qinit):
        hpp = self.client.hpp()
        joints = hpp.robot.getJointNames ()
        self.robotName = joints[0].split('/',1)[0]
        self.objects = { j.split('/',1)[0] for j in joints }
        self.objects.remove (self.robotName)
        self.objectPose = {}
        # Compute objects position
        for jn in joints:
            obj = jn.split('/',1)[0]
            if obj in self.objects:
                iq = self.rankInConfiguration[jn]
                nq = hpp.robot.getJointConfigSize(jn)
                if obj not in self.objectPose:
                    self.objectPose[obj] = qinit[iq:iq+nq]
                else:
                    self.objectPose[obj].extend(qinit[iq:iq+nq])

        rospy.loginfo ("objects pose: " + str(self.objectPose))

    def initializeSoT2HPPconversion (self):
        from agimus_sot_msgs.srv import GetJointNames
        get_joint_names = ros_tools.wait_for_service("/agimus/sot/get_joint_names", GetJointNames)
        self.sot_joint_names = get_joint_names().names
        # Assert that first joint name ends by "root_joint"
        assert self.sot_joint_names[0].endswith("root_joint")

        self.sot2hpp_slices = []
        hpp = self.client.hpp()
        # Special case for the root joint
        rjt = hpp.robot.getJointType(self.robotName + "/root_joint")
        if rjt == "JointModelFreeFlyer":
            self.sot2hpp_rootJointConversion = sotTransRPYToHppPose
        elif rjt == "JointModelPlanar":
            from math import cos, sin
            self.sot2hpp_rootJointConversion = lambda x: list(x[:2]) + [cos(x[5]), sin(x[5])]
        else:
            rospy.logerr("Cannot convert the root joint configuration into HPP root joint type {}."
                    .format(rjt))
        srk = 6
        for sj in self.sot_joint_names[1:]:
            hj = self.robotName + "/" + sj
            if hj not in self.rankInConfiguration:
                rospy.logwarn("SoT joint {} does not correspond to any HPP joint".format(sj))
            else:
                hrk = self.rankInConfiguration[hj]
                hsz = hpp.robot.getJointConfigSize(hj)
                hslice = slice(hrk,hrk+hsz)
                sslice = slice(srk,srk+hsz)
                self.sot2hpp_slices.append ((sslice, hslice))
                srk += hsz

    def __init__ (self):
        from threading import Lock
        # Client to HPP
        self.client = HppClient (context = "simulation")
        self.mutex = Lock()
        # Initialize configuration of robot and objects from HPP
        hpp = self.client.hpp()
        self.q = hpp.problem.getInitialConfig()
        self.configSize = len (self.q)
        # Compute ranks of joints in HPP configuration
        self.computeRanksInConfiguration ()

        self.initializeObjects (self.q)
        self.initializeSoT2HPPconversion()

        self.transitionName = ""
        self.q_rhs = None
        self.objectPublisher = dict ()
        # Create an object publisher by object
        prefix = rospy.get_param ('tf_prefix', 'sim_')
        for o in self.objects:
            self.objectPublisher [o] = PublishObjectPose \
                                       ('world', prefix+o+'/base_link')
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
            rjq = self.sot2hpp_rootJointConversion (msg.data [:6])
            self.q[:len(rjq)] = rjq
            for sslice, hslice in self.sot2hpp_slices:
                self.q[hslice] = msg.data[sslice]
            # update poses of objects
            for o in self.objects:
                pose = self.objectPose [o]
                self.objectPublisher [o].broadcast (pose)
        except Exception:
            rospy.logerr(traceback.format_exc())
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

initialize()
s = Simulation ()
rospy.spin ()
