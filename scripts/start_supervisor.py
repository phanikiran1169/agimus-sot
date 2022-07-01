#!/usr/bin/python

# Copyright 2018 CNRS - Airbus SAS
# Authors: Joseph Mirabel
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

## \package start_supervisor.py
#
#  This node runs a script in the remote python interpreter of the Stack of
#  Tasks. The name of the script is given as input to the node.

from math import sin, cos
import rospy,sys
from std_srvs.srv import Empty
from dynamic_graph_bridge_msgs.srv import RunCommand
from dynamic_graph_bridge_msgs.msg import Vector
from os.path import isfile
from eigenpy import Quaternion, toEulerAngles

def usage():
    rospy.logerr ("Parameters are input (required, a python script), prefix (optional, a string) and"
            " simulate_torque_feedback (optional, boolean which default to true)")
    sys.exit(1)

rospy.init_node ('start_supervisor')

input                                = rospy.get_param("~input",None)
prefix                               = rospy.get_param("~prefix","")
simulateTorqueFeedbackForEndEffector = rospy.get_param("~simulate_torque_feedback",False)

if not input:
    usage()

if not isfile(input):
    raise ValueError ("File not found: " + input)

rospy.loginfo ("Will read file: " + input)

def _runCommandPrint (ans):
    if ans.result != "None":
        rospy.loginfo (ans.result)
    if len(ans.standardoutput) > 0:
        rospy.loginfo (ans.standardoutput)
    if len(ans.standarderror) > 0:
        rospy.logerr (ans.standarderror)

def launchScript(code,title):
    rospy.loginfo(title)
    indent = '  '
    indenting = False
    for line in code:
        if indenting:
            if line == '' or line.startswith(indent):
                rospy.loginfo (".. " + line)
                codeblock += '\n' + line
                continue
            else:
                answer = runCommandClient(str(codeblock))
                _runCommandPrint (answer)
                indenting = False
        if line != '' and line[0] != '#':
            rospy.loginfo (">> " + line)
            if line.endswith(':'):
                indenting = True
                codeblock = line
            else:
                answer = runCommandClient(str(line))
                _runCommandPrint (answer)
    rospy.loginfo("...done with "+title)

def makeRosInterface():
    from agimus_sot.ros_interface import RosInterface
    import rospy
    ri = RosInterface ()
    return ri

# Waiting for services
try:
    rospy.loginfo("Waiting for run_command")
    rospy.wait_for_service('/run_command')
    rospy.loginfo("...ok")

    rospy.loginfo("Waiting for start_dynamic_graph")
    rospy.wait_for_service('/start_dynamic_graph')
    rospy.loginfo("...ok")

    runCommandClient = rospy.ServiceProxy('/run_command', RunCommand)
    runCommandStartDynamicGraph = rospy.ServiceProxy('/start_dynamic_graph', Empty)

    # read ROS param /demo and define dictionary in SoT since SoT is
    # not allowed to import rospy
    demo = rospy.get_param("/demo")
    code = ["globalDemoDict = {}".format(demo)]
    launchScript(code,'define dictionary demoDict')

    initCode = ["simulateTorqueFeedbackForEndEffector = "+str(simulateTorqueFeedbackForEndEffector),] \
            + open( input, "r").read().split("\n")

    rospy.loginfo("Stack of Tasks launched")

    launchScript(initCode,'initialize SoT')
    ri = makeRosInterface ()

    ## \todo this should be moved somewhere else (in agimus).
    ## Initialize pose of robot root_joint
    # read SE(3) pose from ROS parameter
    if rospy.has_param("/robot_initial_pose"):
        rootJointPose = rospy.get_param ("/robot_initial_pose")
        x, y, z, X, Y, Z, W = map (float, rootJointPose.split (' '))

        # read current value of state signal in SoT
        orientation = Quaternion (W, X, Y, Z)
        rz, ry, rx = toEulerAngles (orientation.matrix (), 2, 1, 0)
        code = ["q = robot.device.state.value"]
        code += ["x={0}; y={1}; z={2}".format (x, y, z)]
        code += ["rx={0}; ry={1}; rz={2}".format (rx, ry, rz)]
        code += ["q [0:3] = x, y, z"]
        code += ["q [3:6] = rx, ry, rz"]
        code += ["robot.device.set (q)"]
        launchScript(code,'move robot root_joint to pose specified by ROS' +
                     ' param')

    # request SoT to publish robot state
    ri.publishState (Empty)
    runCommandStartDynamicGraph()

    del runCommandClient
    del runCommandStartDynamicGraph

except rospy.ServiceException as e:
    rospy.logerr("Service call failed: %s" % e)

# We need to spin to provide the SoT ros interface.
try:
    rospy.spin()
except rospy.ROSInterruptException:
    pass
