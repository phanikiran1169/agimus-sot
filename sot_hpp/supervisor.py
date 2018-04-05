from __future__ import print_function
from tools import Manifold, Posture
from dynamic_graph.sot.core import SOT
from dynamic_graph import plug

def _hpTasks (sotrobot):
    return Manifold()
def _lpTasks (sotrobot):
    return Posture ("posture", sotrobot)

class Supervisor(object):
    """
    Steps: P = placement, G = grasp, p = pre-P, g = pre-G
    0. P <-> GP
    1. P <-> gP
    2. gP <-> GP
    3. GP <-> G
    4. GP <-> Gp
    5. Gp <-> G
    """
    def __init__ (self, sotrobot, lpTasks = None, hpTasks = None):
        self.sotrobot = sotrobot
        self.hpTasks = hpTasks if hpTasks is not None else _hpTasks(sotrobot)
        self.lpTasks = lpTasks if lpTasks is not None else _lpTasks(sotrobot)
        self.currentSot = None

    def setupEvents (self):
        from dynamic_graph_hpp.sot import Event, CompareDouble
        from dynamic_graph.sot.core.operator import Norm_of_vector
        from dynamic_graph.ros import RosImport
        self.norm = Norm_of_vector ("control_norm")
        plug (self.sotrobot.device.control, self.norm.sin)

        self.norm_comparision = CompareDouble ("control_norm_comparison")
        plug (self.norm.sout, self.norm_comparision.sin1)
        self.norm_comparision.sin2.value = 1e-2

        self.norm_event = Event ("control_norm_event")
        plug (self.norm_comparision.sout, self.norm_event.condition)
        # self.sotrobot.device.after.addSignal (self.norm_event.check.name)
        self.sotrobot.device.after.addSignal ("control_norm_event.check")

        self.norm_ri = RosImport ('ros_import_control_norm')
        self.norm_ri.add ('double', 'event_control_norm', '/sot_hpp/control_norm_changed')
        plug (self.norm.sout, self.norm_ri.event_control_norm)
        # plug (self.norm_event.trigger, self.norm_ri.trigger)
        self.norm_event.addSignal ("ros_import_control_norm.trigger")

    def makeInitialSot (self):
        # Create the initial sot (keep)
        sot = SOT ('sot_keep')
        sot.setSize(self.sotrobot.dynamic.getDimension())
        self.keep_posture = Posture ("posture_keep", self.sotrobot)
        self.keep_posture.tp.setWithDerivative (False)
        
        # TODO : I do agree that this is a dirty hack.
        # The COM of the robot in the HPP frame is at those coordinates (approx.).
        # But the keep_posture task is « internally » (there is no actuator able to directly move the COM, 
        # but the controller in the task is computing controls anyway, and integrating them) 
        # moving the computed COM to its reference value which is (0, 0, 0).
        # So, when we send the goal coordinates of the feet from HPP to the SoT, there is an offset of 0,74m
        # between the goal and the current position of the feet. This was the cause of the strange feet
        # movements at the beginning of the demo.
        # Maybe we can get the COM position and orientation from HPP at the beginning of the trajectory
        # to initialize self.sotrobot.dynamic.position.value
        # self.keep_posture._signalPositionRef().value = tuple([-0.74, 0.0, 1.0, 0.0, 0.0, 0.0] + list(self.sotrobot.dynamic.position.value)[6:])

        # The above TODO must be fixed in users script by providing the
        # right initial pose using robot.device.set (configuration) before starting
        # dynamic graph.
        self.keep_posture._signalPositionRef().value = self.sotrobot.dynamic.position.value
        
        self.keep_posture.pushTo(sot)
        self.sots[""] = sot

    def topics (self):
        c = self.hpTasks + self.lpTasks
        for g in self.grasps.values():
            c += g

        return c.topics

    def plugTopics (self, rosexport):
        self.rosexport = rosexport
        topics = self.topics()

        for n, t in topics.items():
            if t.has_key('handler'):
                topic = _handlers[t['handler']] (n, t)
            else:
                topic = t["topic"]
            rosexport.add (t["type"], n, topic)
            for s in t['signalGetters']:
                plug (rosexport.signal(n), s())
            print (topic, "plugged to", n, ', ', len(t['signalGetters']), 'times')

    def isSotConsistentWithCurrent(self, transitionName, thr = 1e-3):
        if self.currentSot is None or transitionName == self.currentSot:
            return True
        csot = self.sots[self.currentSot]
        nsot = self.sots[transitionName]
        t = self.sotrobot.device.control.time
        csot.control.recompute(t)
        nsot.control.recompute(t)
        from numpy import array, linalg
        error = array(nsot.control.value) - array(csot.control.value)
        n = linalg.norm(error)
        if n > thr:
            print ("Control not consistent:", linalg.norm(error),'\n', error)
            return False
        return True

    def clearQueues(self):
        exec ("tmp = " + self.rosexport.list())
        for s in tmp:
            self.rosexport.clearQueue(s)

    def readQueue(self, read):
        if read < 0:
            print ("ReadQueue argument should be >= 0")
            return
        t = self.sotrobot.device.control.time
        self.rosexport.readQueue (t + read)

    def stopReadingQueue(self):
        self.rosexport.readQueue (-1)

    def plugSot(self, transitionName, check = False):
        if check and not self.isSotConsistentWithCurrent (transitionName):
            # raise Exception ("Sot %d not consistent with sot %d" % (self.currentSot, id))
            print("Sot {0} not consistent with sot {1}".format(self.currentSot, transitionName))
        if transitionName == "":
            # TODO : Explanation and linked TODO in the function makeInitialSot
            self.keep_posture._signalPositionRef().value = self.sotrobot.dynamic.position.value
        sot = self.sots[transitionName]
        # Start reading queues
        self.readQueue(10)
        plug(sot.control, self.sotrobot.device.control)
        print("Current sot:", transitionName, "\n", sot.display())
        self.currentSot = transitionName

    def runPreAction(self, transitionName):
        if self.preActions.has_key(transitionName):
            sot = self.preActions[transitionName]
            print("Running pre action", transitionName,
                    "\n", sot.display())
            t = self.sotrobot.device.control.time
            sot.control.recompute(t-1)
            plug(sot.control, self.sotrobot.device.control)
            return
        print ("No pre action", transitionName)

    def runPostAction(self, targetStateName):
        if self.postActions.has_key(self.currentSot):
            d = self.postActions[self.currentSot]
            if d.has_key(targetStateName):
                sot = d[targetStateName]
                print( "Running post action", self.currentSot, targetStateName,
                    "\n", sot.display())
                t = self.sotrobot.device.control.time
                sot.control.recompute(t-1)
                plug(sot.control, self.sotrobot.device.control)
                return
        print ("No post action", self.currentSot, targetStateName)

    def getJointList (self, prefix = ""):
        return [ prefix + n for n in self.sotrobot.dynamic.model.names[1:] ]

def _handleHppJoint (n, t):
    type = t["type"]
    if t["velocity"]: topic = "velocity/op_frame"
    else:             topic = "op_frame"
    return "/hpp/target/" + topic + '/' + t['hppjoint']

def _handleHppCom (n, t):
    type = t["type"]
    if t["velocity"]: topic = "velocity/com"
    else:             topic = "com"
    if t['hppcom'] == "":
        return "/hpp/target/" + topic
    else:
        return "/hpp/target/" + topic + '/' + t['hppcom']

_handlers = {
        "hppjoint": _handleHppJoint,
        "hppcom": _handleHppCom,
        }
