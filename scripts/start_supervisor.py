#!/usr/bin/python
import sys, getopt

def usage():
    print ("Usage: " + sys.argv[0] + " --input script.py [--prefix prefix]")
    sys.exit(1)

opts, args = getopt.getopt (sys.argv[1:], "i:p:", ["input=", "prefix="])

f = None
prefix = ""
for opt, arg in opts:
    if opt == "-i" or opt == "--input":
        from os.path import isfile
        if not isfile(arg):
            raise ValueError ("File not found: " + arg)
        f = arg
    elif opt == "-p" or opt == "--prefix":
        prefix = arg
    else:
        usage()

if f is None:
    usage()

import rospy
from std_srvs.srv import *
from dynamic_graph_bridge.srv import *
from dynamic_graph_bridge_msgs.srv import *

rospy.init_node ('start_supervisor')
rospy.loginfo ("Will read file: " + f)

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
    from sot_hpp.ros_interface import RosInterface
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

    initCode = open( f, "r").read().split("\n")

    rospy.loginfo("Stack of Tasks launched")

    launchScript(initCode,'initialize SoT')
    ri = makeRosInterface ()
    runCommandStartDynamicGraph()

except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s" % e)

rospy.spin()
