import rospy
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse, Empty, EmptyResponse
from sot_hpp_msgs.srv import PlugSot, PlugSotResponse, SetString, SetJointNames, GetInt, GetIntResponse, SetInt, SetIntRequest, SetIntResponse
from dynamic_graph_bridge_msgs.srv import RunCommand

def wait_for_service (srv, time = 0.2):
    try:
        rospy.wait_for_service(srv, time)
    except rospy.ROSException:
        rospy.logwarn("Waiting for service: {0}".format(srv))
        rospy.wait_for_service(srv)
    rospy.loginfo("Service {0} found.".format(srv))

class RosInterface(object):
    def __init__ (self, supervisor = None):
        rospy.Service('/sot/plug_sot', PlugSot, self.plugSot)
        rospy.Service('/sot/run_post_action', PlugSot, self.runPostAction)
        rospy.Service('/sot/run_pre_action', PlugSot, self.runPreAction)
        rospy.Service('/sot/request_hpp_topics', Trigger, self.requestHppTopics)
        rospy.Service('/sot/clear_queues', Trigger, self.clearQueues)
        rospy.Service('/sot/read_queue', SetInt, self.readQueue)
        rospy.Service('/sot/stop_reading_queue', Empty, self.stopReadingQueue)
        wait_for_service ("/run_command")
        self._runCommand = rospy.ServiceProxy ('/run_command', RunCommand)
        self.supervisor = supervisor

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
                self.supervisor.runPreAction(req.transition_name)
            except Exception as e:
                rospy.logerr(str(e))
                rsp.success = False
                rsp.msg = str(e)
                return rsp
        else:
            answer = self.runCommand ("supervisor.runPreAction('{}')".format(req.transition_name))
            if len(answer.standarderror) != 0:
                rsp.success = False
                rsp.msg = answer.standarderror
                return rsp
        rsp.success = True
        return rsp

    def plugSot (self, req):
        rsp = PlugSotResponse()
        if self.supervisor is not None:
            try:
                self.supervisor.plugSot(req.transition_name, False)
            except Exception as e:
                rospy.logerr(str(e))
                rsp.success = False
                rsp.msg = str(e)
                return rsp
        else:
            answer = self.runCommand ("supervisor.plugSot('{}', False)".format(req.transition_name))
            if len(answer.standarderror) != 0:
                rsp.success = False
                rsp.msg = answer.standarderror
                return rsp
        rsp.success = True
        return rsp

    def runPostAction (self, req):
        rsp = PlugSotResponse()
        if self.supervisor is not None:
            try:
                self.supervisor.runPostAction(req.transition_name)
            except Exception as e:
                rospy.logerr(str(e))
                rsp.success = False
                rsp.msg = str(e)
                return rsp
        else:
            answer = self.runCommand ("supervisor.runPostAction('{}')".format(req.transition_name))
            if len(answer.standarderror) != 0:
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
            exec ("names = " + answer.result)
        wait_for_service ("/hpp/target/set_joint_names")
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
        return TriggerResponse (True, "ok")

    def readQueue(self, req):
        if self.supervisor is not None:
            self.supervisor.readQueue( req.data)
        else:
            cmd = "supervisor.readQueue(" + str(req.data) + ")"
            answer = self.runCommand (cmd)
        return SetIntResponse ()

    def stopReadingQueue(self, req):
        if self.supervisor is not None:
            self.supervisor.stopReadingQueue ()
        else:
            cmd = "supervisor.stopReadingQueue()"
            answer = self.runCommand (cmd)
        return EmptyResponse ()

    def requestHppTopics(self, req):
        for srv in ['add_center_of_mass', 'add_center_of_mass_velocity', 'add_operational_frame', 'add_operational_frame_velocity',]:
            wait_for_service("/hpp/target/" + srv)
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
            exec ("topics = " + answer.result)
        for n, t in topics.items():
            for k in ['hppjoint', 'hppcom']:
                if t.has_key(k):
                    kk = k if t["velocity"] else ("vel_" + k)
                    handlers[kk] (t[k])
                    rospy.loginfo("Requested " + kk + " " + t[k])
        return TriggerResponse (True, "ok")
