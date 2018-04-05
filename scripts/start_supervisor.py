#!/usr/bin/python
import sys
import rospy

from std_srvs.srv import *
from dynamic_graph_bridge.srv import *
from dynamic_graph_bridge_msgs.srv import *

rospy.init_node ('start_supervisor')

def _runCommandPrint (ans):
    if ans.result != "None":
        rospy.loginfo (ans.result)
    if len(ans.standardoutput) > 0:
        rospy.loginfo (ans.standardoutput)
    if len(ans.standarderror) > 0:
        rospy.logerr (ans.standarderror)

def launchScript(code,title,description = ""):
#    raw_input(title+':   '+description)
    rospy.loginfo(title)
    # rospy.loginfo(code)
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
    ri.setupHppJoints (prefix = "talos/")
    return ri

if len(sys.argv) > 1:
    f = sys.argv[1]
else:
    print ("Usage: " + sys.argv[0] + " script.py")
    sys.exit(1)

# Waiting for services
try:
    rospy.loginfo("Waiting for run_command")
    rospy.wait_for_service('/run_command')
    rospy.loginfo("...ok")

    rospy.loginfo("Waiting for start_dynamic_graph")
    rospy.wait_for_service('/start_dynamic_graph')
    rospy.loginfo("...ok")

    runCommandClient = rospy.ServiceProxy('run_command', RunCommand)
    runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)

    initCode = open( f, "r").read().split("\n")

    rospy.loginfo("Stack of Tasks launched")

    launchScript(initCode,'initialize SoT')
#    raw_input("Wait before starting ros interface")
    ri = makeRosInterface ()
#    raw_input("Wait before starting the dynamic graph")
    runCommandStartDynamicGraph()

except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s" % e)

rospy.spin()
