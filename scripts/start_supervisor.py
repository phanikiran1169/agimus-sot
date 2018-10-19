#!/usr/bin/python

import rospy,sys
from std_srvs.srv import *
from dynamic_graph_bridge.srv import *
from dynamic_graph_bridge_msgs.srv import *
from os.path import isfile

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
    ri.setupHppJoints (prefix = prefix)
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

    initCode = ["simulateTorqueFeedbackForEndEffector = "+str(simulateTorqueFeedbackForEndEffector),] \
            + open( input, "r").read().split("\n")

    rospy.loginfo("Stack of Tasks launched")

    launchScript(initCode,'initialize SoT')
    ri = makeRosInterface ()
    runCommandStartDynamicGraph()

except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s" % e)

# We need to spin to provide the SoT ros interface.
rospy.spin()
