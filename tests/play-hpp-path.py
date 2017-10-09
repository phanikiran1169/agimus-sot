#!/usr/bin/python
import sys
import rospy

from std_srvs.srv import *
from dynamic_graph_bridge.srv import *
from dynamic_graph_bridge_msgs.srv import *

def launchScript(code,title,description = ""):
    raw_input(title+':   '+description)
    rospy.loginfo(title)
    rospy.loginfo(code)
    indent = '  '
    indenting = False
    for line in code:
        if indenting:
            if line == '' or line.startswith(indent):
                codeblock += '\n' + line
                continue
            else:
                print codeblock
                answer = runCommandClient(str(codeblock))
                rospy.logdebug(answer)
                print answer
                indenting = False
        if line != '' and line[0] != '#':
            if line.endswith(':'):
                indenting = True
                codeblock = line
            else:
                print line
                answer = runCommandClient(str(line))
                rospy.logdebug(answer)
                print answer
    rospy.loginfo("...done with "+title)


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

    initCode = open( "appli-play-hpp-path.py", "r").read().split("\n")

    rospy.loginfo("Stack of Tasks launched")


    launchScript(initCode,'initialize SoT')
    raw_input("Wait before starting the dynamic graph")
    runCommandStartDynamicGraph()
    raw_input("Wait before pushing the posture task in SoT")
    runCommandClient("sot.push(taskPosture.name)")
    runCommandClient("robot.device.control.recompute(0)")

    raw_input("Wait before starting the seqplay")
    runCommandClient("aPlayHppPath.start()")

except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s" % e)

