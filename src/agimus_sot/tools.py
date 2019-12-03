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

from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph.sot.core.meta_tasks_kine_relative import MetaTaskKine6dRel
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core import FeaturePosture
from dynamic_graph import plug
import numpy as np

def getTimerType (type):
    from dynamic_graph.sot.core.timer import TimerDouble, TimerMatrix, TimerMatrixHomo, TimerVector
    if type == "double":
        return TimerDouble
    elif type == "matrix":
        return TimerMatrix
    elif type == "matrixhomo":
        return TimerMatrixHomo
    elif type == "vector":
        return TimerVector
    else:
        raise ValueError ("Unknown type of timer.")

def filename_escape(value):
    """
    Normalizes string, converts to lowercase, removes non-alpha characters,
    and converts spaces to hyphens.
    """
    import unicodedata, re
    value = unicodedata.normalize('NFKD', unicode(value)).encode('ascii', 'ignore')
    value = unicode(re.sub('[^\w\s-]', '', value).strip().lower())
    value = unicode(re.sub('[-\s]+', '-', value))
    return str(value)

def insertTimerOnOutput (signal, type):
    """
    Plug the signal sout of the return entity instead of `signal` to
    input signal to enable the timer.
    - param signal an output signal.
    - return an Timer entity.
    """
    Timer = getTimerType (type)
    timer = Timer ("timer_of_" + signal.name)
    plug(signal, timer.sin)
    return timer

def insertTimer (signal, type):
    """
    - param signal a plugged input signal.
    - return an Timer entity.
    """
    assert signal.isPlugged()
    from dynamic_graph.sot.core.timer import TimerDouble, TimerMatrix, TimerMatrixHomo, TimerVector
    Timer = getTimerType (type)
    timer = Timer ("timer_of_" + signal.name)
    other = signal.getPlugged()
    plug(other, timer.sin)
    plug(timer.sout, signal)
    return timer

def parseHppName (hppjointname):
    if hppjointname == "universe": return "", "universe"
    return hppjointname.split('/', 1)

def transQuatToSE3 (p):
    from pinocchio import SE3, Quaternion
    from numpy import matrix
    return SE3 (Quaternion (p[6],p[3],p[4],p[5]).matrix(), matrix(p[0:3]).transpose())

def se3ToTuple (M):
    from dynamic_graph.sot.core.matrix_util import matrixToTuple
    return matrixToTuple (M.homogeneous)

def computeControlSelection (robot, joint_to_be_removed):
    pinmodel = robot.dynamic.model
    selection = ["1",] * pinmodel.nv
    for j in filter(lambda x: pinmodel.names[x.id] in joint_to_be_removed, pinmodel.joints[1:]):
        selection[j.idx_v:j.idx_v+j.nv] = ["0",] * j.nv
    selection.reverse()
    return "".join(selection)

def _createOpPoint (robot, name):
    if not robot.dynamic.hasSignal(name):
        robot.dynamic.createOpPoint(name, name)

def plugMatrixHomo(sigout, sigin):
    from dynamic_graph.signal_base import SignalBase
    from pinocchio import SE3
    if isinstance(sigout, tuple):
        sigin.value = sigout
    elif isinstance(sigout, SE3):
        sigin.value = se3ToTuple(sigout)
    elif isinstance(sigout, SignalBase):
        plug(sigout, sigin)

## \todo this should move to dynamic-graph-python
def assertEntityDoesNotExist(name):
    from dynamic_graph.entity import Entity
    assert name not in Entity.entities, "Entity " + name + " already exists."

def matrixHomoProduct(name, *args):
    from dynamic_graph.sot.core import Multiply_of_matrixHomo
    assertEntityDoesNotExist(name)
    ent = Multiply_of_matrixHomo (name)
    ent.setSignalNumber(len(args))
    for i, valueOrSignal in enumerate(args):
        if valueOrSignal is None: continue
        plugMatrixHomo (valueOrSignal, ent.signal('sin'+str(i)))
    return ent

def matrixHomoInverse(name, valueOrSignal=None):
    from dynamic_graph.sot.core import Inverse_of_matrixHomo
    assertEntityDoesNotExist(name)
    ent = Inverse_of_matrixHomo (name)
    plugMatrixHomo(valueOrSignal, ent.sout)
    return ent

## Abstraction of a task.
#
# Each child class sets a default value for the gain.
class Manifold(object):
    sep = "___"

    def __init__ (self, tasks = [], constraints = [], topics = {}):
        ## Task to be added to a SoT solver
        self.tasks = list(tasks)
        ## Constraints
        # This is likely not used anymore.
        self.constraints = list(constraints)
        ## The ROS topics where to read references.
        # It is a dictionary. The key is the output signal name of a
        # RosQueuedSubscribe entity. The value is a dictionary with:
        # \li "type": "vector", "matrixHomo", "vector3" ...,
        # \li "topic": the ROS topic name,
        # \li "signalGetters": a list of functions returning the input signals to be plugged.
        # \li "handler": [optional] some specific topics have specific handlers.
        #     - "hppjoint": needs two more keys
        #       - "velocity": boolean
        #       - "hppjoint": HPP joint name
        #     - "tf_listener": needs three more keys
        #       - "velocity": boolean
        #       - "frame0": base frame name
        #       - "frame1": child frame name
        #     - "hppcom": needs two more keys
        #       - "velocity": boolean
        #       - "hppcom": HPP CoM name
        #
        #
        self.topics = dict(topics)

    def __add__ (self, other):
        res = Manifold(list(self.tasks), list(self.constraints), dict(self.topics))
        res += other
        return res

    def __iadd__ (self, other):
        self.tasks += other.tasks
        self.constraints += other.constraints
        for k,v in other.topics.items():
            if self.topics.has_key(k):
                a = self.topics[k]
                assert a["type"] == v["type"]
                if a.has_key('topic'): assert a["topic"] == v["topic"]
                else: assert a["handler"] == v["handler"]
                self.extendSignalGetters(k, v["signalGetters"])
                # print k, "has", len(a["signalGetters"]), "signals"
            else:
                self.topics[k] = v
        return self

    def setControlSelection (self, selection):
        for t in self.tasks:
            t.controlSelec.value = selection

    def pushTo (self, solver):
        """
        \param solver an object of type agimus_sot.solver.Solver
        """
        for t in self.tasks:
            solver.sot.push(t.name)
            solver.tasks.append(t)

    def extendSignalGetters (self, topicName, signalGetters):
        """Add signal getters to a topic"""
        sgs = signalGetters if isinstance(signalGetters, (list, tuple, set, frozenset)) else [signalGetters,]
        topic =  self.topics[topicName]
        topic["signalGetters"] = topic["signalGetters"].union (sgs)

    def addHppJointTopic (self, topicName, jointName=None, velocity=False, signalGetters=frozenset()):
        """
        Add a topic that will received the pose (or velocity) of a joint from HPP
        - param jointName: When None, uses topicName as the joint name in HPP.
        """
        if topicName in self.topics:
            tp=self.topics[topicName]
            assert "velocity" in tp and tp['velocity'] ==  velocity
            assert "type"     in tp and tp["type"    ] ==  "matrixHomo"
            assert "handler"  in tp and tp["handler" ] ==  "hppjoint"
            assert "hppjoint" in tp and tp["hppjoint"] ==  jointName if jointName is not None else topicName
            self.extendSignalGetters(topicName, signalGetters)
            return
        self.topics[topicName] = {
                "velocity": velocity,
                "type": "vector" if velocity else "matrixHomo",
                "handler": "hppjoint",
                "hppjoint": jointName if jointName is not None else topicName,
                "signalGetters": frozenset(signalGetters),
                }

    def addTfListenerTopic (self, topicName, frame0, frame1,
            defaultValue=None, signalGetters=frozenset(),
            maxDelay=0.3):
        if topicName in self.topics:
            tp=self.topics[topicName]
            assert "velocity" in tp and tp['velocity'] ==  False
            assert "type"     in tp and tp["type"    ] ==  "matrixHomo"
            assert "handler"  in tp and tp["handler" ] ==  "tf_listener"
            assert "frame0"   in tp and tp["frame0"  ] ==  frame0
            assert "frame1"   in tp and tp["frame1"  ] ==  frame1
            assert "maxDelay" in tp and tp["maxDelay"] ==  maxDelay
            self.extendSignalGetters(topicName, signalGetters)
            return
        self.topics[topicName] = {
                "velocity": False,
                "type": "matrixHomo",
                "handler": "tf_listener",
                "frame0": frame0,
                "frame1": frame1,
                "signalGetters": frozenset(signalGetters),
                "maxDelay": maxDelay
                }
        if defaultValue is not None:
            self.topics[topicName]["defaultValue"] = defaultValue

## Postural task
class Posture(Manifold):
    def __init__ (self, name, sotrobot, withDerivative = False):
        super(Posture, self).__init__()
        from dynamic_graph.sot.core import Task, FeatureGeneric, GainAdaptive

        n = Posture.sep + name
        self.tp = Task ('task' + n)
        self.tp.dyn = sotrobot.dynamic
        self.tp.feature = FeaturePosture('feature_'+n)

        q = list(sotrobot.dynamic.position.value)
        self.tp.feature.state.value = q
        self.tp.feature.posture.value = q

        robotDim = sotrobot.dynamic.getDimension()
        for i in range(6, robotDim):
            self.tp.feature.selectDof (i, True)
        self.tp.gain = GainAdaptive("gain_"+n)
        self.tp.add(self.tp.feature.name)

        # Connects the dynamics to the current feature of the posture task
        plug(sotrobot.dynamic.position, self.tp.feature.state)

        self.tp.setWithDerivative (withDerivative)

        # Set the gain of the posture task
        setGain(self.tp.gain,(4.9,0.9,0.01,0.9))
        plug(self.tp.gain.gain, self.tp.controlGain)
        plug(self.tp.error, self.tp.gain.error)
        self.tasks = [ self.tp ]
        self.topics = {
                    name: {
                        "type": "vector",
                        "topic": "/hpp/target/position",
                        "signalGetters": frozenset([ self._signalPositionRef ]) },
                }
        if withDerivative:
            self.topics["vel_" + name] = {
                    "type": "vector",
                    "topic": "/hpp/target/velocity",
                    "signalGetters": frozenset([ self._signalVelocityRef ])
                    }

    def _signalPositionRef (self): return self.tp.feature.posture
    def _signalVelocityRef (self): return self.tp.feature.postureDot

## Represents a gripper or a handle
class OpFrame(object):
    def __init__ (self, srdf, model = None, enabled = None):
        self.robotName = srdf["robot"]
        self.name = srdf["name"]
        self.key = self.robotName + "/" + self.name
        self.link = srdf["link"]
        self.lMf = transQuatToSE3 (srdf["position"])
        self.enabled = enabled
        if srdf.has_key ("joints"):
            assert model is not None
            ## Only for grippers
            self.joints = srdf["joints"]
            self._setupParentJoint (self.link, self.lMf, model)
            if self.enabled is None:
                self.enabled = True
            if srdf.has_key("torque_constant"):
                self.torque_constant = srdf["torque_constant"]
        else:
            ## Only for handles
            # kept for backward compat
            self.pose = self.lMf
            if self.enabled is None:
                self.enabled = False
        # TODO See note in README.md
        self.hasVisualTag = False

    def _setupParentJoint (self, link, pose, model):
        frameid = model.getFrameId (link)
        if frameid < 0 or frameid >= len(model.frames):
            raise ValueError("Link " + self.link + " not found")
        frame = model.frames[frameid]

        # kept for backward compat
        self.pose = frame.placement * pose
        self.jMf = self.pose
        self.joint = model.names[frame.parent]

    @property
    def fullLink  (self): return self.robotName + "/" + self.link
    @property
    def fullJoint (self): return self.robotName + "/" + self.joint
    @property
    def fullName  (self): return self.robotName + "/" + self.name

## \brief A pregrasp (and preplace) task.
# It creates a task to pose of the gripper with respect to the handle.
# This pose should follow the relative pose from planning.
# If the \c handle is attached to the joint tree (by another grasp) then,
# provide the \c otherGraspOnObject.
#
# There are 3 cases:
# - \c gripper enabled and \c otherGripper disabled or not provided:
#   Absolute pose of the \c gripper with respect to the \c handle.
# - \c gripper and \c otherGripper enabled:
#   Relative pose of the \c gripper with respect to the \c otherGripper.
# - \c gripper disabled and \c otherGripper enabled:
#   Absolute pose of the \c otherGripper with respect to the \c handle.
#
# \note For preplace task, the gripper is on the environment surface and
# the handle is on the object surface.
class PreGrasp (Manifold):
    ## Constructor
    # \param gripper object of type OpFrame
    # \param handle object of type OpFrame
    # \param otherGraspOnObject either None or a tuple (otherGripper, otherHandle)
    def __init__ (self, gripper, handle, otherGraspOnObject = None):
        super(PreGrasp, self).__init__()
        self.gripper = gripper
        self.handle = handle
        if otherGraspOnObject is not None:
            self.otherGripper = otherGraspOnObject[0]
            self.otherHandle  = otherGraspOnObject[1]
        else:
            self.otherGripper = None
            self.otherHandle  = None

    def makeTasks(self, sotrobot, withMeasurementOfObjectPos, withMeasurementOfGripperPos,
            withMeasurementOfOtherGripperPos = False, withDerivative = False):
        if self.gripper.enabled:
            if self.otherGripper is not None and self.otherGripper.enabled:
                self._makeRelativeTask (sotrobot,
                        withMeasurementOfObjectPos, withMeasurementOfGripperPos,
                        withMeasurementOfOtherGripperPos, withDerivative)
            else:
                self._makeAbsolute (sotrobot,
                        withMeasurementOfObjectPos, withMeasurementOfGripperPos,
                        withDerivative)
        else:
            if self.otherGripper is not None and self.otherGripper.enabled:
                self._makeAbsoluteBasedOnOther (sotrobot,
                        withMeasurementOfObjectPos, withMeasurementOfGripperPos,
                        withMeasurementOfOtherGripperPos,
                        withDerivative)
            else:
                # TODO Both grippers are disabled so nothing can be done...
                # add a warning ?
                print("Both grippers are disabled so nothing can be done")

    ## Plug the position of linkName to \c outSignal.
    #  The pose of linkName must be computable by the SoT robot entity.
    def _plugRobotLink (self, sotrobot, linkName, poseSignal, Jsignal, withMeasurement):
        if withMeasurement:
            self.addTfListenerTopic(linkName + "_measured",
                    frame0 = "world",
                    frame1 = linkName + "_measured",
                    defaultValue = sotrobot.dynamic.signal(linkName),
                    signalGetters = [poseSignal,],
                    )
        else:
            plug(sotrobot.dynamic.signal(linkName), poseSignal)
        if Jsignal is not None:
            plug(sotrobot.dynamic.signal("J"+linkName), Jsignal)

    ## Plug the position of linkName to \c outSignal.
    #  The pose of linkName is not computable by the SoT robot entity.
    #  \warning The topic linkName must have been created before.
    #  \todo add the ability to plug the TF listener default value to
    #        the HPP joint topic called linkName.
    def _plugObjectLink (self, linkName, outSignal, withMeasurement):
        if withMeasurement:
            self.addTfListenerTopic (linkName + "_measured",
                    frame0 = "world",
                    frame1 = linkName + "_measured",
                    signalGetters = [outSignal,],
                    # TODO
                    # defaultValue = ...,
                    )
        else:
            self.extendSignalGetters(linkName, outSignal)

    ## Compute desired pose between gripper and handle.
    #  It is decomposed as \f$ jgMg^-1 * oMjg^-1 * oMlh * lhMh \f$.
    #  It creates the entity faMfbDes.
    #  Topic \c handle.fullLink must exists.
    def _referenceSignal (self, name, gripper, handle):
        # oMjg^-1 -> HPP joint
        self.oMjaDes_inv = matrixHomoInverse (name + "_oMjaDes_inv")
        self.addHppJointTopic (gripper.fullLink, signalGetters = [ self.oMjaDes_inv.sin, ],)
        # Plug it to FeaturePose
        self.faMfbDes = matrixHomoProduct (name + "_faMfbDes",
            gripper.lMf.inverse(), # jgMg^-1
            self.oMjaDes_inv.sout, # oMjg^-1 -> HPP joint
            None,                  # oMlh -> HPP joint
            handle.lMf,            # lhMh
            )
        # oMlh -> HPP joint
        self.extendSignalGetters(handle.fullLink, self.faMfbDes.sin2)

    def _createTaskAndGain (self, name):
        # Create a task
        from dynamic_graph.sot.core import Task, GainAdaptive
        self.task = Task (name + "_task")
        self.task.add (self.feature.name)

        # Set the task gain
        self.gain = GainAdaptive(name + "_gain")
        setGain(self.gain,(4.9,0.9,0.01,0.9))
        plug(self.gain.gain, self.task.controlGain)
        plug(self.task.error, self.gain.error)

    ## \todo implement tracking of velocity
    def _makeAbsolute(self, sotrobot, withMeasurementOfObjectPos, withMeasurementOfGripperPos, withDerivative):
        name = PreGrasp.sep.join(["", "pregrasp", self.gripper.name, self.handle.fullName])

        from dynamic_graph.sot.core.feature_pose import FeaturePose
        assertEntityDoesNotExist(name+"_feature")
        self.feature = FeaturePose (name + "_feature")

        # Create the operational points
        _createOpPoint (sotrobot, self.gripper.link)

        self._plugRobotLink (sotrobot, self.gripper.link,
                self.feature.oMja, self.feature.jaJja,
                withMeasurementOfGripperPos)
        self.feature.jaMfa.value = se3ToTuple(self.gripper.lMf)

        self.addHppJointTopic (self.handle.fullLink)
        self._plugObjectLink (self.handle.fullLink,
                self.feature.oMjb, withMeasurementOfObjectPos)
        self.feature.jbMfb.value = se3ToTuple(self.handle.lMf)
        self.feature.jbJjb.value = np.zeros((6, sotrobot.dynamic.getDimension()))

        # Compute desired pose between gripper and handle.
        # Creates the entity faMfbDes
        self._referenceSignal (name, self.gripper, self.handle)
        plug(self.faMfbDes.sout, self.feature.faMfbDes)

        # Create a task and gain
        self._createTaskAndGain (name)

        if withDerivative:
            print("Relative pose constraint with derivative is not implemented yet.")
        self.task.setWithDerivative (False)

        self.tasks = [ self.task, ]

    ## \todo implement tracking of velocity
    def _makeRelativeTask (self, sotrobot,
            withMeasurementOfObjectPos, withMeasurementOfGripperPos,
            withMeasurementOfOtherGripperPos,
            withDerivative):
        assert self.handle.robotName == self.otherHandle.robotName
        assert self.handle.link      == self.otherHandle.link
        name = PreGrasp.sep.join(["", "pregrasp", self.gripper.name, self.handle.fullName,
            "relative", self.otherGripper.name, self.otherHandle.fullName])

        from dynamic_graph.sot.core.feature_pose import FeaturePose
        assertEntityDoesNotExist(name+"_feature")
        self.feature = FeaturePose (name + "_feature")

        # Create the operational points
        _createOpPoint (sotrobot, self.     gripper.link)
        _createOpPoint (sotrobot, self.otherGripper.link)

        # Joint A is the gripper link
        self._plugRobotLink (sotrobot, self.     gripper.link,
                self.feature.oMja, self.feature.jaJja,
                withMeasurementOfGripperPos)
        # Frame A is the gripper frame
        self.feature.jaMfa.value = se3ToTuple(self.gripper.lMf)

        # Joint B is the other gripper link
        self._plugRobotLink (sotrobot, self.otherGripper.link,
                self.feature.oMjb, self.feature.jbJjb,
                withMeasurementOfOtherGripperPos)

        # Frame B is the handle frame
        # jbMfb = ogMh = ogMo(t) * oMh
        method = 2
        if method == 0: # Works
            # jbMfb        = ogMoh * ohMo * oMh
            self.feature.jbMfb.value = se3ToTuple (
                    self.otherGripper.lMf
                    * self.otherHandle.lMf.inverse()
                    * self.handle.lMf)
            self.addHppJointTopic (self.handle.fullLink)
        elif method == 1: # Does not work
            # Above, it is assumed that ogMoh = Id, which must be corrected.
            # Instead, we compute
            # jbMfb = ogMh = ogMo(t) * oMh
            #              = wMog(t)^-1 * wMo(t) * oMh
            # wMog^-1
            self.wMog_inv = matrixHomoInverse (name + "_wMog_inv")
            self._plugRobotLink (sotrobot, self.otherGripper.link,
                    self.wMog_inv.sin, None,
                    withMeasurementOfOtherGripperPos)
            self.jbMfb = matrixHomoProduct (name + "_jbMfb",
                self.wMog_inv.sout,      # wMog^-1 -> HPP joint
                None,                    # wMo -> HPP joint
                self.handle.lMf,         # oMh
                )
            # wMo
            self.addHppJointTopic (self.handle.fullLink)
            self._plugObjectLink (self.handle.fullLink,
                    self.jbMfb.sin1, withMeasurementOfObjectPos)
            # Plug it to FeaturePose
            plug(self.jbMfb.sout, self.feature.jbMfb)
        elif method == 2: # Seems to work
            # jbMfb        = ogMo * oMh
            self.jbMfb = matrixHomoProduct (name + "_jbMfb",
                None,                    # ogMo -> HPP joint
                self.handle.lMf,         # oMh
                )
            plug(self.jbMfb.sout, self.feature.jbMfb)
            # ogMo
            self.addTfListenerTopic (
                    self.otherHandle.fullLink + "_wrt_" + self.otherGripper.link + "_measured",
                    frame0 = self.otherGripper.link + "_measured",
                    frame1 = self.otherHandle.fullLink + "_measured",
                    defaultValue = se3ToTuple (self.otherGripper.lMf * self.otherHandle.lMf.inverse()),
                    signalGetters = [ self.jbMfb.sin0, ],
                    )

            self.addHppJointTopic (self.handle.fullLink)

        # Compute desired pose between gripper and handle.
        # Creates the entity faMfbDes
        self._referenceSignal (name, self.gripper, self.handle)
        plug(self.faMfbDes.sout, self.feature.faMfbDes)

        # Create a task and gain
        self._createTaskAndGain (name)

        if withDerivative:
            print("Relative pose constraint with derivative is not implemented yet.")
        self.task.setWithDerivative (False)

        self.tasks = [ self.task ]
        # TODO Add velocity

    ## \todo implement tracking of velocity
    def _makeAbsoluteBasedOnOther (self, sotrobot,
            withMeasurementOfObjectPos, withMeasurementOfGripperPos,
            withMeasurementOfOtherGripperPos, withDerivative):
        assert self.handle.robotName == self.otherHandle.robotName
        assert self.handle.link      == self.otherHandle.link
        name = PreGrasp.sep.join(["", "pregrasp", self.gripper.fullName, self.handle.fullName,
            "based", self.otherGripper.name, self.otherHandle.fullName])

        from dynamic_graph.sot.core.feature_pose import FeaturePose
        assertEntityDoesNotExist(name+"_feature")
        self.feature = FeaturePose (name + "_feature")

        # Create the operational point
        _createOpPoint (sotrobot, self.otherGripper.link)

        # Joint A is the gripper link
        self.addHppJointTopic (self.gripper.fullLink)
        self._plugObjectLink (self.gripper.fullLink,
                self.feature.oMja, withMeasurementOfGripperPos)
        # Frame A is the gripper frame
        self.feature.jaMfa.value = se3ToTuple(self.gripper.lMf)
        self.feature.jaJja.value = np.zeros((6, sotrobot.dynamic.getDimension()))

        # Joint B is the other gripper link
        self._plugRobotLink (sotrobot, self.otherGripper.link,
                self.feature.oMjb, self.feature.jbJjb,
                withMeasurementOfOtherGripperPos)
        # Frame B is the handle frame
        method = 1
        if method == 0: # Does not work
            # jbMfb = ogMh = ogMw * wMo * oMh
            # wMog^-1
            self.wMog_inv = matrixHomoInverse (name + "_wMog_inv")
            self._plugRobotLink (sotrobot, self.otherGripper.link,
                    self.wMog_inv.sin, None,
                    withMeasurementOfOtherGripperPos)
            self.jbMfb = matrixHomoProduct (name + "_jbMfb",
                self.wMog_inv.sout,      # wMog^-1 -> HPP joint
                None,                    # wMo -> HPP joint
                self.handle.lMf,         # oMh
                )
            # wMo
            self.addHppJointTopic (self.handle.fullLink)
            self._plugObjectLink (self.handle.fullLink,
                    self.jbMfb.sin1, withMeasurementOfObjectPos)
            # Plug it to FeaturePose
            plug(self.jbMfb.sout, self.feature.jbMfb)
        elif method == 1:
            # jbMfb        = ogMo * oMh
            self.jbMfb = matrixHomoProduct (name + "_jbMfb",
                None,                    # ogMo -> TF
                self.handle.lMf,         # oMh
                )
            plug(self.jbMfb.sout, self.feature.jbMfb)
            # ogMo
            self.addTfListenerTopic (
                    self.otherHandle.fullLink + "_wrt_" + self.otherGripper.link + "_measured",
                    frame0 = self.otherGripper.link + "_measured",
                    frame1 = self.otherHandle.fullLink + "_measured",
                    defaultValue = se3ToTuple (self.otherGripper.lMf * self.otherHandle.lMf.inverse()),
                    signalGetters = [ self.jbMfb.sin0, ],
                    )

            self.addHppJointTopic (self.handle.fullLink)

        # Compute desired pose between gripper and handle.
        # Creates the entity faMfbDes
        self._referenceSignal (name, self.gripper, self.handle)
        plug(self.faMfbDes.sout, self.feature.faMfbDes)

        # Create a task and gain
        self._createTaskAndGain (name)

        if withDerivative:
            print("Relative pose constraint with derivative is not implemented yet.")
        self.task.setWithDerivative (False)

        self.tasks = [ self.task ]

## A grasp task
# It creates a grasp constraint only in the case where
# an object is grasped by two grippers.
class Grasp (Manifold):
    def __init__ (self, gripper, handle, otherGraspOnObject = None):
        super(Grasp, self).__init__()
        self.gripper = gripper
        self.handle = handle

        # self.relative = otherGraspOnObject is not None \
                # and otherGraspOnObject.handle.link == self.handle.link
        # TODO: We should make sure that the relative position is constant
        # but we have no way to do it (HPP does). We assume that the object is
        # not articulated.
        self.relative = (otherGraspOnObject is not None)
        if self.relative:
            self.otherGripper = otherGraspOnObject[0]
            self.otherHandle = otherGraspOnObject[1]

    ## \todo implement tracking of velocity
    # \c self.task.task.setWithDerivative(True)
    # which needs the following signals:
    # - velocity of object attached to gripper.sotjoint, into self.task.feature.dotposition
    # - velocity of object attached to otherGripper.sotjoint, into self.task.feature.dotpositionRef
    def makeTasks(self, sotrobot, withDerivative = False):
        if self.relative:
            basename = Grasp.sep.join([self.gripper.name, self.otherGripper.name])

            # Create the FeaturePose
            from dynamic_graph.sot.core.feature_pose import FeaturePose
            self.feature = FeaturePose (basename+"_feature")
            #self.feature.jaMfa.value = \
            #        se3ToTuple(self.otherGripper.pose * self.otherHandle.pose.inverse())
            #self.feature.jbMfb.value = \
            #        se3ToTuple(self.gripper     .pose * self.handle     .pose.inverse())

            # Create the operational points
            _createOpPoint (sotrobot, self.     gripper.link)
            _createOpPoint (sotrobot, self.otherGripper.link)

            dyn = sotrobot.dynamic
            plug(dyn.signal(    self.otherGripper.link), self.feature. oMja)
            plug(dyn.signal(    self.     gripper.link), self.feature. oMjb)
            plug(dyn.signal('J'+self.otherGripper.link), self.feature.jaJja)
            plug(dyn.signal('J'+self.     gripper.link), self.feature.jbJjb)

            self.feature.faMfbDes.value = \
                    se3ToTuple(self.otherHandle .lMf
                            *  self.otherGripper.lMf.inverse()
                            *  self.     gripper.lMf
                            *  self.     handle .lMf.inverse())

            # Wrap it into a Task
            from dynamic_graph.sot.core import Task, GainAdaptive
            self.task = Task (basename+"_task")
            self.gain = GainAdaptive (basename+"_gain")

            self.task.add (self.feature.name)
            plug (self.task.error, self.gain.error)
            plug (self.gain.gain , self.task.controlGain)

            setGain(self.gain,(4.9,0.9,0.01,0.9))
            if withDerivative:
                print("Grasp constraint with derivative is not implemented yet.")
            self.task.setWithDerivative (False)

            self.tasks = [ self.task ]


## Control of the gripper motors.
#
# It can do:
# \li position control
# \li admittance control
# \li a mixture between position and admittance control (position control before the impact, then admittance control).
#     \todo At the time of writting, I (Joseph Mirabel) think the mixtured control is bugged. At least, there was a SEGV the last time
#     I tried to use it. However, the SEGV might have a different cause.
class EndEffector (Manifold):
    def __init__ (self, sotrobot, gripper, name_suffix):
        from dynamic_graph.sot.core import Task, GainAdaptive

        super(EndEffector, self).__init__()
        self.gripper = gripper
        self.jointNames = gripper.joints
        self.robot = sotrobot
        pinmodel = sotrobot.dynamic.model

        self.name = Posture.sep.join(["ee", gripper.name, name_suffix])

        self.tp = Task ('task' + self.name)
        self.tp.dyn = sotrobot.dynamic
        self.tp.feature = FeaturePosture ('feature_' + self.name)

        plug(sotrobot.dynamic.position, self.tp.feature.state)

        # Select the dofs
        self.tp.feature.posture.value = sotrobot.dynamic.position.value
        # Define the reference and the selected DoF
        self.jointRanks = []
        for name in self.jointNames:
            idJ = pinmodel.getJointId(name)
            assert idJ < pinmodel.njoints
            joint = pinmodel.joints[idJ]
            idx_v = joint.idx_v
            nv = joint.nv
            self.jointRanks.append( (idx_v, nv) )
        for idx_v, nv in self.jointRanks:
            for i in range(idx_v, idx_v + nv):
                self.tp.feature.selectDof (i, True)

        self.tp.gain = GainAdaptive("gain_"+self.name)
        self.tp.add(self.tp.feature.name)

        # Set the gain of the posture task
        setGain(self.tp.gain,(4.9,0.9,0.01,0.9))
        plug(self.tp.gain.gain, self.tp.controlGain)
        plug(self.tp.error, self.tp.gain.error)
        if len(self.jointNames) > 0:
            self.tasks = [ self.tp ]

        self.thr_task_error = 0.0001

    ### \param type equals "open" or "close"
    ### \param period interval between two integration of SoT
    def makeAdmittanceControl (self, affordance, type, period,
            simulateTorqueFeedback = False,
            filterCurrents = True):
        # Make the admittance controller
        from agimus_sot.control.gripper import AdmittanceControl, PositionAndAdmittanceControl
        from .events import norm_superior_to
        # type = "open" or "close"
        desired_torque = affordance.ref["torque"]
        estimated_theta_close = affordance.ref["angle_close"]

        wn, z, nums, denoms = affordance.getControlParameter ()

        if affordance.controlType[type] == "torque":
            self.ac = AdmittanceControl ("AC_" + self.name + "_" + type,
                    estimated_theta_close,
                    desired_torque, period,
                    nums, denoms)
        elif affordance.controlType[type] == "position_torque":
            theta_open = affordance.ref["angle_open"]
            threshold_up = tuple([ x / 10. for x in desired_torque ])
            threshold_down = tuple([ x / 100. for x in desired_torque ])
            self.ac = PositionAndAdmittanceControl ("AC_" + self.name + "_" + type,
                    theta_open, estimated_theta_close,
                    desired_torque, period,
                    threshold_up, threshold_down,
                    wn, z,
                    nums, denoms)
        else:
            raise NotImplementedError ("Control type " + type + " is not implemented for gripper.")

        if simulateTorqueFeedback:
            # Get torque from an internal simulation
            M,d,k,x0 = affordance.getSimulationParameters()
            self.ac.setupFeedbackSimulation(M,d,k,x0)
            # Must be done after setupFeedbackSimulation
            self.ac.readPositionsFromRobot(self.robot, self.jointNames)
        else:
            self.ac.readPositionsFromRobot(self.robot, self.jointNames)
            # Get torque from robot sensors (or external simulation)
            # TODO allows to switch between current and torque sensors
            self.ac.readCurrentsFromRobot(self.robot, self.jointNames,
                    (self.gripper.torque_constant,), filterCurrents)
            # self.ac.readTorquesFromRobot(self.robot, self.jointNames)

        from dynamic_graph.sot.core.operator import Mix_of_vector
        mix_of_vector = Mix_of_vector (self.name + "_control_to_robot_control")
        mix_of_vector.default.value = tuple ([0,] * len(self.tp.feature.posture.value))
        mix_of_vector.setSignalNumber(2)
        for idx_v,nv in self.jointRanks:
            mix_of_vector.addSelec(1, idx_v, nv)

        # Plug the admittance controller to the posture task
        setGain(self.tp.gain,1.)
        plug(self.ac.outputPosition, mix_of_vector.signal("sin1"))
        plug(mix_of_vector.sout, self.tp.feature.posture)
        # TODO plug posture dot ?
        # I do not think it is necessary.
        # TODO should we send to the posture task
        # - the current robot posture
        # - the postureDot from self.ac.outputDerivative
        # This would avoid the last integration in self.ac.
        # This integration does not know the initial point so
        # there might be some drift (removed the position controller at the beginning)

        from .events import norm_superior_to, norm_inferior_to, logical_and_entity
        tnorm, tcomp = norm_superior_to (self.name + "_torquecmp",
                self.ac.currentTorqueIn, 0.95 * np.linalg.norm(desired_torque))
        pnorm, pcomp = norm_inferior_to (self.name + "_positioncmp",
                self.tp.error, self.thr_task_error)
        self.events = {
                "done_close": logical_and_entity (self.name + '_done_close_and', [tcomp.sout, pcomp.sout]),
                }

    def makePositionControl (self, position):
        q = list(self.tp.feature.state.value)
        # Define the reference
        ip = 0
        for iq, nv in self.jointRanks:
            q[iq:iq+nv] = position[ip:ip+nv]
            ip += nv
        assert ip == len(position)
        self.tp.feature.posture.value = q
        setGain(self.tp.gain,(4.9,0.9,0.01,0.9))

        from .events import norm_inferior_to
        n, c = norm_inferior_to (self.name + "_positioncmp",
                self.tp.error, self.thr_task_error)
        self.events = {
                "done_close": c.sout,
                "done_open": c.sout,
                }

class Foot (Manifold):
    def __init__ (self, footname, sotrobot, selec='111111', withDerivative = False):
        super(Foot, self).__init__()

        robotname, sotjoint = parseHppName (footname)
        basename = Foot.sep + footname

        # Create the FeaturePose
        from dynamic_graph.sot.core.feature_pose import FeaturePose
        self.feature = FeaturePose ('pose'+basename)

        dyn = sotrobot.dynamic 
        if sotjoint not in dyn.signals():
            dyn.createOpPoint (sotjoint, sotjoint)
        plug(dyn.signal(    sotjoint), self.feature. oMjb)
        plug(dyn.signal('J'+sotjoint), self.feature.jbJjb)

        # Wrap it into a Task
        from dynamic_graph.sot.core import Task, GainAdaptive
        self.task    = Task ('task'+basename)
        self.gain    = GainAdaptive ('gain'+basename)

        self.task.add (self.feature.name)
        plug (self.task.error, self.gain.error)
        plug (self.gain.gain , self.task.controlGain)

        setGain(self.gain,(4.9,0.9,0.01,0.9))
        self.task.setWithDerivative (withDerivative)

        if selec!='111111':
            self.feature.selec.value = selec

        self.tasks = [ self.task, ]
        self.addHppJointTopic (footname, signalGetters=[ self._signalPositionRef ])
        if withDerivative:
            self.addHppJointTopic ("vel_" + footname, footname,
                    velocity = True,
                    signalGetters=[ self._signalVelocityRef ])

    def _signalPositionRef (self): return self.feature.faMfbDes
    def _signalVelocityRef (self): return self.feature.faNufafbDes

class COM (Manifold):
    sep = "_com_"
    def __init__ (self, comname, sotrobot, withDerivative = False):
        self.taskCom = MetaTaskKineCom (sotrobot.dynamic,
                name = COM.sep + comname)
        super(COM, self).__init__()
        self.taskCom.task.setWithDerivative (withDerivative)
        self.tasks = [ self.taskCom.task, ]
        self.topics[comname] = {
                "velocity": False,
                "type": "vector3",
                "handler": "hppcom",
                "hppcom": comname,
                "signalGetters": frozenset([ self._signalPositionRef ])
                }
        if withDerivative:
            self.topics["vel_" + comname] = {
                    "velocity": True,
                    "type": "vector3",
                    "handler": "hppcom",
                    "hppcom": comname,
                    "signalGetters": frozenset([ self._signalVelocityRef ])
                    }
        self.taskCom.task.controlGain.value = 5

    def _signalPositionRef (self): return self.taskCom.featureDes.errorIN
    def _signalVelocityRef (self): return self.taskCom.featureDes.errordotIN
