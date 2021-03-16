# Copyright 2018 CNRS - Airbus SAS
# Author: Joseph Mirabel and Alexis Nicolin
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

import rospy
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse, Empty, EmptyResponse
from agimus_sot_msgs.srv import PlugSot, PlugSotResponse, GetJointNames, ReadQueue, WaitForMinQueueSize, WaitForMinQueueSizeResponse, SetPose
from dynamic_graph_bridge_msgs.srv import RunCommand

def wait_for_service (srv, time = 0.2):
    try:
        rospy.wait_for_service(srv, time)
        rospy.loginfo("Service {0} found.".format(srv))
    except rospy.ROSException:
        rospy.logwarn("Waiting for service: {0}".format(srv))
        rospy.wait_for_service(srv)
        rospy.logwarn("Service {0} found.".format(srv))

## Ros interface for \ref supervisor.Supervisor.
#
# There are two ways of communicating with SoT.
# \li use this class directly inside the python interpreter in SoT,
# \li use this class in a separate ROS node that communicates with SoT
#     via the service "/run_command".
#
# The first method can be useful for debugging. However, **on the robot,
# only the second method should be used**.
class RosInterface(object):
    ## \param supervisor if None, then communication with SoT is handled via
    #
    # \todo Service "run_command" should be used only when supervisor is None.
    def __init__ (self, supervisor = None):
        rospy.Service('plug_sot', PlugSot, self.plugSot)
        rospy.Service('run_post_action', PlugSot, self.runPostAction)
        rospy.Service('run_pre_action', PlugSot, self.runPreAction)
        rospy.Service('request_hpp_topics', Trigger, self.requestHppTopics)
        rospy.Service('clear_queues', Trigger, self.clearQueues)
        rospy.Service('wait_for_min_queue_size', WaitForMinQueueSize, self.waitForMinQueueSize)
        rospy.Service('read_queue', ReadQueue, self.readQueue)
        rospy.Service('stop_reading_queue', Empty, self.stopReadingQueue)
        rospy.Service('publish_state', Empty, self.publishState)
        rospy.Service('set_base_pose', SetPose, self.setBasePose)
        rospy.Service('get_joint_names', GetJointNames, self.getJointNames)
        wait_for_service ("/run_command")
        self._runCommand = rospy.ServiceProxy ('/run_command', RunCommand)
        self.supervisor = supervisor

    def _isNotError (self, runCommandAnswer):
        if len(runCommandAnswer.standarderror) != 0:
            return False, runCommandAnswer.standarderror
        return True, ""

    def runCommand (self, cmd):
        rospy.loginfo (">> " + cmd)
        answer = self._runCommand (cmd)
        if answer.result != "None":
            rospy.loginfo (answer.result)
        if len(answer.standardoutput) > 0:
            rospy.loginfo (answer.standardoutput)
        if len(answer.standarderror) > 0:
            rospy.logerr (answer.standarderror)
        return answer

    def runPreAction (self, req):
        rsp = PlugSotResponse()
        if self.supervisor is not None:
            try:
                rsp.success, rsp.start_time = self.supervisor.runPreAction(req.transition_name)
            except Exception as e:
                rospy.logerr(str(e))
                rsp.success = False
                rsp.msg = str(e)
                return rsp
        else:
            answer = self.runCommand ("supervisor.runPreAction('{}')".format(req.transition_name))
            rsp.success, rsp.msg = self._isNotError (answer)
            if rsp.success:
                exec ("rsp.success, rsp.start_time = " + answer.result)
            else:
                return rsp
        rsp.msg = "Successfully called supervisor."
        return rsp

    def plugSot (self, req):
        rsp = PlugSotResponse()
        if self.supervisor is not None:
            try:
                rsp.success, rsp.start_time = self.supervisor.plugSot(req.transition_name, False)
            except Exception as e:
                rospy.logerr(str(e))
                rsp.success = False
                rsp.msg = str(e)
                return rsp
        else:
            answer = self.runCommand ("supervisor.plugSot('{}', False)".format(req.transition_name))
            rsp.success, rsp.msg = self._isNotError (answer)
            if rsp.success:
                exec ("rsp.success, rsp.start_time = " + answer.result)
            else:
                return rsp
        return rsp

    def runPostAction (self, req):
        rsp = PlugSotResponse()
        if self.supervisor is not None:
            try:
                rsp.success, rsp.start_time = self.supervisor.runPostAction(req.transition_name)
            except Exception as e:
                rospy.logerr(str(e))
                rsp.success = False
                rsp.msg = str(e)
                return rsp
        else:
            answer = self.runCommand ("supervisor.runPostAction('{}')".format(req.transition_name))
            rsp.success, rsp.msg = self._isNotError (answer)
            if rsp.success:
                exec ("rsp.success, rsp.start_time = " + answer.result)
            else:
                return rsp
        rsp.msg = "Successfully called supervisor."
        return rsp

    def getJointNames(self, req):
        if self.supervisor is not None:
            names = self.supervisor.getJointList()
        else:
            answer = self.runCommand ("supervisor.getJointList()")

            success, message = self._isNotError (answer)
            if success:
                exec ("names = " + answer.result)
            else:
                rospy.logerr("Could not get the joint names\n" + message)
        return (names,)

    def clearQueues(self, req):
        if self.supervisor is not None:
            self.supervisor.clearQueues()
        else:
            cmd = "supervisor.clearQueues()"
            answer = self.runCommand (cmd)
        return TriggerResponse (True, "ok")

    def readQueue(self, req):
        from agimus_sot_msgs.srv import ReadQueueResponse
        rsp = ReadQueueResponse()
        if self.supervisor is not None:
            rsp.success, rsp.start_time = self.supervisor.readQueue(req.delay, req.minQueueSize, req.duration, req.timeout)
        else:
            cmd = "supervisor.readQueue({},{},{},{})".format(req.delay, req.minQueueSize, req.duration, req.timeout)
            answer = self.runCommand (cmd)
            rsp.success, rsp.message = self._isNotError (answer)
            if rsp.success:
                exec ("rsp.success, rsp.start_time = " + answer.result)
            else:
                return rsp
        if not rsp.success:
            rsp.message = "Timeout reached"
        return rsp

    def waitForMinQueueSize(self, req):
        from agimus_sot_msgs.srv import WaitForMinQueueSizeResponse
        rsp = WaitForMinQueueSizeResponse()
        if self.supervisor is not None:
            rsp.success, rsp.message = self.supervisor.waitForQueue(req.minQueueSize, req.timeout)
        else:
            cmd = "supervisor.waitForQueue({},{})".format(req.minQueueSize, req.timeout)
            answer = self.runCommand (cmd)
            rospy.loginfo(answer.standarderror)
            rsp.success, rsp.message = self._isNotError (answer)
            if rsp.success:
                exec ("rsp.success, rsp.message = " + answer.result)
        return rsp

    def stopReadingQueue(self, req):
        if self.supervisor is not None:
            self.supervisor.stopReadingQueue ()
        else:
            cmd = "supervisor.stopReadingQueue()"
            answer = self.runCommand (cmd)
        return EmptyResponse ()

    def publishState(self, req):
        if self.supervisor is not None:
            self.supervisor.publishState ()
        else:
            cmd = "supervisor.publishState()"
            answer = self.runCommand (cmd)
        return EmptyResponse ()

    def requestHppTopics(self, req):
        for srv in ['add_center_of_mass', 'add_center_of_mass_velocity', 'add_operational_frame', 'add_operational_frame_velocity',]:
            wait_for_service("/hpp/target/" + srv)

        from agimus_sot_msgs.srv import SetString
        handlers = {
                'hppcom': rospy.ServiceProxy ('/hpp/target/add_center_of_mass', SetString),
                'vel_hppcom': rospy.ServiceProxy ('/hpp/target/add_center_of_mass_velocity', SetString),
                'hppjoint': rospy.ServiceProxy ('/hpp/target/add_operational_frame', SetString),
                'vel_hppjoint': rospy.ServiceProxy ('/hpp/target/add_operational_frame_velocity', SetString),
                }
        if self.supervisor is not None:
            topics = self.supervisor.topics()
        else:
            cmd = "{ n: { k: v for k, v in t.items() if k in ['hppjoint', 'hppcom', 'velocity'] } for n, t in supervisor.topics().items() }"
            answer = self.runCommand (cmd)
            exec ("topics = " + answer.result)
        for n, t in topics.items():
            for k in ['hppjoint', 'hppcom']:
                if t.has_key(k):
                    kk = k if not t["velocity"] else ("vel_" + k)
                    handlers[kk] (t[k])
                    rospy.loginfo("Requested " + kk + " " + t[k])
        # This sleep is mandatory to let some time to ROS to connect
        # topic publisher and susbcriber. Otherwise, the first message is dropped.
        rospy.sleep(1)
        return TriggerResponse (True, "ok")

    def setBasePose (self, req):
        pose = [ req.x, req.y, req.z, req.roll, req.pitch, req.yaw ]
        if self.supervisor is not None:
            try:
                self.supervisor.setBasePose(pose)
            except Exception as e:
                rospy.logerr(str(e))
                return False, str(e)
        else:
            answer = self.runCommand ("supervisor.setBasePose({})".format(pose))
            if len(answer.standarderror) != 0:
                return False, answer.standarderror
        return True, ""
