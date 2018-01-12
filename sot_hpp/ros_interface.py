import rospy
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse, Empty, EmptyResponse
from sot_hpp_msgs.srv import PlugSot, PlugSotResponse, SetString, SetJointNames, GetInt, GetIntResponse, SetInt, SetIntRequest, SetIntResponse
from dynamic_graph_bridge_msgs.srv import RunCommand

class RosInterface(object):
    def __init__ (self, supervisor = None):
        rospy.Service('/sot/plug_sot', PlugSot, self.plugSot)
        rospy.Service('/sot/run_post_action', PlugSot, self.runPostAction)
        rospy.Service('/sot/run_pre_action', PlugSot, self.runPreAction)
        rospy.Service('/sot/request_hpp_topics', Trigger, self.requestHppTopics)
        rospy.Service('/sot/clear_queues', Trigger, self.clearQueues)
        rospy.Service('/sot/read_queue', SetInt, self.readQueue)
        rospy.Service('/sot/stop_reading_queue', Empty, self.stopReadingQueue)
        self.runCommand = rospy.ServiceProxy ('/run_command', RunCommand)
        self.supervisor = supervisor

    def runPreAction (self, req):
        rsp = PlugSotResponse()
        if self.supervisor is not None:
            try:
                self.supervisor.runPreAction(req.transition_id)
            except Exception as e:
                rospy.logerr(str(e))
                rsp.success = False
                rsp.msg = str(e)
                return rsp
        else:
            answer = self.runCommand ("supervisor.runPreAction({})".format(req.transition_id))
            if len(answer.standarderror) != 0:
                rospy.logerr(answer.standarderror)
                rsp.success = False
                rsp.msg = answer.standarderror
                return rsp
        rsp.success = True
        return rsp

    def plugSot (self, req):
        rsp = PlugSotResponse()
        if self.supervisor is not None:
            try:
                self.supervisor.plugSot(req.transition_id, False)
            except Exception as e:
                rospy.logerr(str(e))
                rsp.success = False
                rsp.msg = str(e)
                return rsp
        else:
            answer = self.runCommand ("supervisor.plugSot({}, False)".format(req.transition_id))
            if len(answer.standarderror) != 0:
                rospy.logerr(answer.standarderror)
                rsp.success = False
                rsp.msg = answer.standarderror
                return rsp
        rsp.success = True
        return rsp

    def runPostAction (self, req):
        rsp = PlugSotResponse()
        if self.supervisor is not None:
            try:
                self.supervisor.runPostAction(req.transition_id)
            except Exception as e:
                rospy.logerr(str(e))
                rsp.success = False
                rsp.msg = str(e)
                return rsp
        else:
            answer = self.runCommand ("supervisor.runPostAction({})".format(req.transition_id))
            if len(answer.standarderror) != 0:
                rospy.logerr(answer.standarderror)
                rsp.success = False
                rsp.msg = answer.standarderror
                return rsp
        rsp.success = True
        return rsp

    def setupHppJoints(self, prefix = ""):
        if self.supervisor is not None:
            names = self.supervisor.getJointList(prefix = prefix)
        else:
            answer = self.runCommand ("supervisor.getJointList(prefix = '{}')".format(prefix))
            print answer
            exec ("names = " + answer.result)
        setJoints = rospy.ServiceProxy ('/hpp/target/set_joint_names', SetJointNames)
        ans = setJoints (names)
        if not ans.success:
            rospy.logerr("Could not set the joint list of hpp_ros_interface node")

    def clearQueues(self, req):
        if self.supervisor is not None:
            self.supervisor.clearQueues()
        else:
            cmd = "supervisor.clearQueues()"
            answer = self.runCommand (cmd)
            print cmd
            print answer
        return TriggerResponse (True, "ok")

    def readQueue(self, req):
        if self.supervisor is not None:
            self.supervisor.readQueue( req.data)
        else:
            cmd = "supervisor.readQueue(" + str(req.data) + ")"
            answer = self.runCommand (cmd)
            print cmd
            print answer
        return SetIntResponse ()

    def stopReadingQueue(self, req):
        if self.supervisor is not None:
            self.supervisor.stopReadingQueue ()
        else:
            cmd = "supervisor.stopReadingQueue()"
            answer = self.runCommand (cmd)
            print cmd
            print answer
        return EmptyResponse ()

    def requestHppTopics(self, req):
        handlers = {
                'hppcom': rospy.ServiceProxy ('/hpp/target/add_center_of_mass', SetString),
                'vel_hppcom': rospy.ServiceProxy ('/hpp/target/add_center_of_mass_velocity', SetString),
                'hppjoint': rospy.ServiceProxy ('/hpp/target/add_operational_frame', SetString),
                'vel_hppjoint': rospy.ServiceProxy ('/hpp/target/add_operational_frame_velocity', SetString),
                }
        if self.supervisor is not None:
            topics = self.supervisor.topics()
        else:
            cmd = "{ n: { k: v for k, v in t.items() if k != 'signalGetters' } for n, t in supervisor.topics().items() }"
            answer = self.runCommand (cmd)
            print cmd
            print answer
            exec ("topics = " + answer.result)
        for n, t in topics.items():
            for k in ['hppjoint', 'hppcom']:
                if t.has_key(k):
                    kk = k if t["velocity"] else ("vel_" + k)
                    handlers[kk] (t[k])
                    rospy.loginfo("Requested " + kk + " " + t[k])
        return TriggerResponse (True, "ok")
