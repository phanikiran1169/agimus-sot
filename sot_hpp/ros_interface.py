import rospy
from std_srvs.srv import Trigger, TriggerResponse
from hpp_ros_interface.srv import PlugSot, PlugSotResponse, SetString, SetJointNames
from dynamic_graph_bridge_msgs.srv import RunCommand

class RosInterface(object):
    def __init__ (self, supervisor = None):
        rospy.Service('/sot/plug_sot', PlugSot, self.plugSot)
        rospy.Service('/sot/request_hpp_topics', Trigger, self.requestHppTopics)
        self.runCommand = rospy.ServiceProxy ('/run_command', RunCommand)
        self.supervisor = supervisor

    def plugSot (self, req):
        rsp = PlugSotResponse()
        if self.supervisor is not None:
            try:
                self.supervisor.plugSot(req.transition_id, True)
            except Exception as e:
                rsp.success = False
                rsp.msg = str(e)
                return rsp
        else:
            answer = self.runCommand ("supervisor.plugSot({}, True)".format(req.transition_id))
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
