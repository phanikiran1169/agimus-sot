# Copyright 2018 CNRS - Airbus SAS
# Authors: Joseph Mirabel and Alexis Nicolin
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

from __future__ import print_function
from .task import Task, Posture
from dynamic_graph import plug
from dynamic_graph.sot.core.feature_posture import FeaturePosture
import sys

def _hpTasks (sotrobot):
    return Task()
def _lpTasks (sotrobot):
    return Posture ("posture", sotrobot, True)

## Supervise the consecutive execution of several SoT.
#
# Typically, these sots are created via factory.Factory. They can also be added manually.
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
    ##
    # \param lpTasks list of low priority tasks. If None, a Posture task will be used.
    # \param hpTasks list of high priority tasks (like balance)
    def __init__ (self, sotrobot, lpTasks=None, hpTasks=None, prefix=None):
        self.sotrobot = sotrobot
        if prefix is None:
            self.prefix = sotrobot.name + "/"
        else:
            self.prefix = prefix + "/"
        self.hpTasks = hpTasks if hpTasks is not None else _hpTasks(sotrobot)
        self.lpTasks = lpTasks if lpTasks is not None else _lpTasks(sotrobot)
        self.currentSot = None
        from dynamic_graph.sot.core.switch import SwitchVector
        self.sot_switch = SwitchVector ("sot_supervisor_switch")
        plug(self.sot_switch.sout, self.sotrobot.device.control)

        from agimus_sot.events import Events
        self. done_events = Events ("done" , sotrobot)
        self.error_events = Events ("error", sotrobot)
        self. done_events.setupNormOfControl (sotrobot.device.control, 1e-2)
        self. done_events.setupTime () # For signal self. done_events.timeEllapsedSignal
        self.error_events.setupTime () # For signal self.error_events.timeEllapsedSignal

    def makeInitialSot (self):
        # Create the initial sot (keep)
        from .solver import Solver
        sot = Solver ('sot_keep', self.sotrobot.dynamic.getDimension())

        self.keep_posture = Posture ("posture_keep", self.sotrobot)
        self.keep_posture._task.setWithDerivative (False)
        self.keep_posture._signalPositionRef().value = self.sotrobot.dynamic.position.value

        self.keep_posture.pushTo(sot)
        sot. doneSignal = self.done_events.controlNormSignal
        sot.errorSignal = False
        self.addSolver ("", sot)

    ## Set the robot base pose in the world.
    # \param basePose a list: [x,y,z,r,p,y] or [x,y,z,qx,qy,qz,qw]
    # \return success True in case of success
    def setBasePose (self, basePose):
        if len(basePose) == 7:
            # Currently, this case never happens
            from dynamic_graph.sot.tools.quaternion import Quaternion
            from numpy.linalg import norm
            q = Quaternion(basePose[6],basePose[3],basePose[4],basePose[5])
            if abs(norm(q.array) - 1.) > 1e-2:
              return False, "Quaternion is not normalized"
            basePose = basePose[:3] + q.toRPY().tolist()
        if self.currentSot == "" or len(basePose) != 6:
            # We are using the SOT to keep the current posture.
            # The 6 first DoF are not used by the task so we can change them safely.
            q = self.sotrobot.device.state.value
            q[:6] = basePose
            self.sotrobot.device.set(q)
            self.keep_posture._signalPositionRef().value = self.sotrobot.device.state.value
            return True
        else:
            return False

    ## \name SoT managements
    ##  \{

    def addPreAction (self, name, preActionSolver):
        self.preActions[name] = preActionSolver
        self._addSignalToSotSwitch (preActionSolver)

    def addSolver (self, name, solver):
        self.sots[name] = solver
        self._addSignalToSotSwitch (solver)

    def duplicateSolver (self, existingSolver, newSolver):
        self.sots[newSolver] = self.sots[existingSolver]

    def addPostActions (self, name, postActionSolvers):
        self.postActions[name] = postActionSolvers
        for targetState, pa_sot in postActionSolvers.iteritems():
            self._addSignalToSotSwitch (pa_sot)

    ## This is for internal purpose
    def _addSignalToSotSwitch (self, solver):
        n = self.sot_switch.getSignalNumber()
        self.sot_switch.setSignalNumber(n+1)
        self.sots_indexes[solver.name] = n
        plug (solver.control, self.sot_switch.signal("sin" + str(n)))

        def _plug (e, events, n, name):
            assert events.getSignalNumber() == n, "Wrong number of events."
            events.setSignalNumber(n+1)
            events.setConditionString(n, name)
            if isinstance(e, (bool,int)): events.conditionSignal(n).value = e
            else: plug (e, events.conditionSignal(n))

        _plug (solver. doneSignal, self. done_events, n, solver.name)
        _plug (solver.errorSignal, self.error_events, n, solver.name)

    def _selectSolver (self, solver):
        n = self.sots_indexes[solver.name]
        self.  sot_switch.selection.value = n
        self. done_events.setSelectedSignal(n)
        self.error_events.setSelectedSignal(n)

    ## \}

    def topics (self):
        c = self.hpTasks + self.lpTasks
        for g in self.grasps.values():
            c += g
        for p in self.placements.values():
            c += p

        return c.topics

    def plugTopicsToRos (self):
        from dynamic_graph.ros.ros_queued_subscribe import RosQueuedSubscribe
        self.rosSubscribe = RosQueuedSubscribe ('ros_queued_subscribe')
        from dynamic_graph.ros.ros_tf_listener import RosTfListener
        self.rosTf = RosTfListener ('ros_tf_listener')
        topics = self.topics()

        for name, topic_info in topics.items():
            topic_handler = _handlers[topic_info.get("handler","default")]
            topic_handler (name,topic_info,self.rosSubscribe,self.rosTf)

    def printQueueSize (self):
        for l in self.rosSubscribe.list():
            print (l, self.rosSubscribe.queueSize(l))

    ## Check consistency between two SoTs.
    #
    # This is not used anymore because it must be synchronized with the real-time thread.
    # \todo Re-enable consistency check between two SoTs.
    def isSotConsistentWithCurrent(self, transitionName, thr = 1e-3):
        if self.currentSot is None or transitionName == self.currentSot:
            return True
        csot = self.sots[self.currentSot]
        nsot = self.sots[transitionName]
        t = self.sotrobot.device.control.time
        # This is not safe since it would be run concurrently with the
        # real time thread.
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
        self.rosSubscribe.readQueue (-1)
        for s in self.rosSubscribe.list():
            print ('{} queue size: {}'.format(s, self.rosSubscribe.queueSize(s)))
            self.rosSubscribe.clearQueue(s)

    ## Wait for the queue to be of a given size.
    # \param minQueueSize (integer) waits to the queue size of rosSubscribe
    #                     to be greater or equal to \c minQueueSize
    # \param timeout time in seconds after which to return a failure.
    # \return True on success, False on timeout.
    def waitForQueue(self, minQueueSize, timeout):
        ts = self.sotrobot.device.getTimeStep()
        to = int(timeout / self.sotrobot.device.getTimeStep())
        from time import sleep
        start_it = self.sotrobot.device.control.time
        for queue in self.rosSubscribe.list():
            while self.rosSubscribe.queueSize(queue) < minQueueSize:
                if self.sotrobot.device.control.time > start_it + to:
                    return False, "Queue {} has received {} points.".format(queue, self.rosSubscribe.queueSize(queue))
                sleep(ts)
        return True, ""

    ## Start reading values received by the RosQueuedSubscribe entity.
    # \param delay (integer) how many periods to wait before reading.
    #              It allows to give some delay to network connection.
    # \param minQueueSize (integer) waits to the queue size of rosSubscribe
    #                     to be greater or equal to \p minQueueSize
    # \param duration expected duration (in seconds) of the queue.
    # \param timeout time in seconds after which to return a failure.
    # \return success, time boolean, SoT time at which reading starts (invalid if success is False)
    #
    # \warning If \p minQueueSize is greater than the number of values to
    #          be received by rosSubscribe, this function does an infinite loop.
    def readQueue(self, delay, minQueueSize, duration, timeout):
        from time import sleep
        print("Current solver {0}".format(self.currentSot))
        if delay < 0:
            print ("Delay argument should be >= 0")
            return False, -1
        minSizeReached, msg = self.waitForQueue (minQueueSize, timeout)
        if not minSizeReached:
            print (msg)
            return False, -1
        durationStep = int(duration / self.sotrobot.device.getTimeStep())
        t = self.sotrobot.device.control.time + delay
        self.rosSubscribe.readQueue (t)
        self. done_events.setFutureTime (t + durationStep)
        self.error_events.setFutureTime (t + durationStep)
        return True, t

    def stopReadingQueue(self):
        self.rosSubscribe.readQueue (-1)

    # \return success, time boolean, SoT time at which reading starts (invalid if success is False)
    def plugSot(self, transitionName, check = False):
        if check and not self.isSotConsistentWithCurrent (transitionName):
            # raise Exception ("Sot %d not consistent with sot %d" % (self.currentSot, id))
            print("Sot {0} not consistent with sot {1}".format(self.currentSot, transitionName))
        if transitionName == "":
            # Retrieve the posture feature common to all SoT
            f = FeaturePosture(self.lpTasks._feature.name)
            # Set reference posture as the latest reference read from
            # the reference trajectory topic.
            if len(f.posture.value > 0):
                self.keep_posture._signalPositionRef().value = f.posture.value
            else:
                # reference of posture feature has not been initialized yet
                self.keep_posture._signalPositionRef().value = \
                    self.sotrobot.dynamic.position.value
        solver = self.sots[transitionName]

        # No done events should be triggered before call
        # to readQueue. We expect it to happen with 1e6 milli-seconds
        # from now...
        devicetime = self.sotrobot.device.control.time
        self. done_events.setFutureTime (devicetime + 100000)

        self._selectSolver (solver)
        print("{0}: Current solver {1}\n{2}"
                .format(devicetime, transitionName, solver.sot.display()))
        self.currentSot = transitionName
        if hasattr (self, 'ros_publish_state'):
            self.ros_publish_state.signal("transition_name").value = transitionName
        return True, devicetime

    # \return success, time boolean, SoT time at which reading starts (invalid if success is False)
    def runPreAction(self, transitionName):
        if self.preActions.has_key(transitionName):
            solver = self.preActions[transitionName]

            t = self.sotrobot.device.control.time + 2
            self. done_events.setFutureTime (t)

            self._selectSolver (solver)
            print("{0}: Running pre action {1}\n{2}"
                    .format(t, transitionName, solver.sot.display()))
            return True, t - 2
        print ("No pre action", transitionName)
        return False, -1

    ## Execute a post-action
    # \return success, time boolean, SoT time at which reading starts (invalid if success is False)
    def runPostAction(self, targetStateName):
        if self.postActions.has_key(self.currentSot):
            d = self.postActions[self.currentSot]
            if d.has_key(targetStateName):
                solver = d[targetStateName]

                devicetime = self.sotrobot.device.control.time
                self. done_events.setFutureTime (devicetime + 2)

                self._selectSolver (solver)

                print("{0}: Running post action {1} --> {2}\n{3}"
                        .format(devicetime, self.currentSot, targetStateName,
                            solver.sot.display()))
                return True, devicetime
        print ("No post action {0} --> {1}".format(self.currentSot, targetStateName))
        return False, -1

    def getJointList (self):
        return [self.prefix + n for n in self.sotrobot.dynamic.model.names[1:]]

    def publishState (self, subsampling = 40):
        if hasattr (self, "ros_publish_state"):
            return
        from dynamic_graph.ros import RosPublish
        self.ros_publish_state = RosPublish ("ros_publish_state")
        self.ros_publish_state.add ("vector", "state", "/agimus/sot/state")
        self.ros_publish_state.add ("vector", "reference_state", "/agimus/sot/reference_state")
        self.ros_publish_state.add ("string", "transition_name",
                                    "/agimus/sot/transition_name")
        self.ros_publish_state.signal("transition_name").value = ""
        plug (self.sotrobot.device.state, self.ros_publish_state.signal("state"))
        plug (self.rosSubscribe.signal("posture"), self.ros_publish_state.signal("reference_state"))
        self.sotrobot.device.after.addDownsampledSignal ("ros_publish_state.trigger", subsampling)

def _defaultHandler(name,topic_info,rosSubscribe,rosTf):
    topic = topic_info["topic"]
    rosSubscribe.add (topic_info["type"], name, topic)
    for s in topic_info['signalGetters']:
        from dynamic_graph.signal_base import SignalBase
        plug (rosSubscribe.signal(name), s if isinstance(s, SignalBase) else s())
    print (topic, "plugged to", name, ', ', len(topic_info['signalGetters']), 'times')

def _handleTfListener (name,topic_info,rosSubscribe,rosTf):
    from dynamic_graph.signal_base import SignalBase
    signame = topic_info["frame1"] + "_wrt_" + topic_info["frame0"]
    rosTf.add (topic_info["frame0"], topic_info["frame1"], signame)
    for t in topic_info['signalGetters']:
        if isinstance(t, SignalBase):
            plug (rosTf.signal(signame), t)
        elif isinstance(t, (list, tuple)) and len(t) == 2:
            plug (rosTf.signal(signame), t[0])
            plug (rosTf.signal(signame+"_available"), t[1])
        else:
            raise TypeError("Expect a signal or tuple of two signals")
    if "defaultValue" in topic_info:
        dv = topic_info["defaultValue"]
        if isinstance(dv, SignalBase):
            plug(dv, rosTf.signal(signame+"_failback"))
        else:
            rosTf.signal(signame+"_failback").value = dv
    if "maxDelay" in topic_info:
        rosTf.setMaximumDelay (signame, topic_info["maxDelay"])
    print (topic_info["frame1"], "wrt", topic_info["frame0"], "plugged to", signame, ', ', len(topic_info['signalGetters']), 'times')

def _handleHppJoint (name,topic_info,rosSubscribe,rosTf):
    if topic_info["velocity"]: topic = "velocity/op_frame"
    else:                      topic = "op_frame"
    ti = dict(topic_info)
    ti["topic"] = "/hpp/target/" + topic + '/' + topic_info['hppjoint']
    _defaultHandler (name,ti,rosSubscribe,rosTf)

def _handleHppCom (name,topic_info,rosSubscribe,rosTf):
    if topic_info["velocity"]: topic = "velocity/com"
    else:                      topic = "com"
    ti = dict(topic_info)
    if topic_info['hppcom'] == "":
        ti["topic"] = "/hpp/target/" + topic
    else:
        ti["topic"] = "/hpp/target/" + topic + '/' + topic_info['hppcom']
    _defaultHandler (name,ti,rosSubscribe,rosTf)

_handlers = {
        "hppjoint": _handleHppJoint,
        "hppcom": _handleHppCom,
        "tf_listener": _handleTfListener,
        "default": _defaultHandler,
        }
