import rospy
from std_srvs.srv import Trigger, TriggerResponse
from hpp_ros_interface.srv import PlugSot, PlugSotResponse, SetString, SetJointNames

class RosInterface(object):
    def __init__ (self, supervisor):
        self.supervisor = supervisor
        rospy.Service('/sot/plug_sot', PlugSot, self.plugSot)
        rospy.Service('/sot/request_hpp_topics', Trigger, self.requestHppTopics)

    def plugSot (self, req):
        rsp = PlugSotResponse()
        try:
            status = self.supervisor.isSotConsistentWithCurrent (req.transition_id)
            if not status:
                rsp.msg = "Sot %d not consistent with sot %d" % (self.supervisor.currentSot, req.transition_id)
        except Exception, e:
            rospy.logerr(str(e))
            rsp.success = False
            rsp.msg = str(e)
            return rsp
        try:
            self.supervisor.plugSot(req.transition_id)
            rsp.success = True
        except Exception, e:
            msg = str(e)
            rsp.success = False
        return rsp

    def setupHppJoints(self, prefix = None):
        names = [ prefix + n for n in self.supervisor.sotrobot.dynamic.model.names[2:] ]
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
        topics = self.supervisor.topics()
        for n, t in topics.items():
            for k in ['hppjoint', 'hppcom']:
                if t.has_key(k):
                    kk = k if t["velocity"] else ("vel_" + k)
                    handlers[kk] (t[k])
                    rospy.loginfo("Requested " + kk + " " + t[k])
        return TriggerResponse (True, "ok")
