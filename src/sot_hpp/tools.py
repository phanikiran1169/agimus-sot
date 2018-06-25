from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph.sot.core.meta_tasks_kine_relative import MetaTaskKine6dRel
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core import FeaturePosture, Multiply_of_matrixHomo, Inverse_of_matrixHomo
from dynamic_graph import plug

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

class Manifold(object):
    sep = "___"

    def __init__ (self, tasks = [], constraints = [], topics = {}, initial_control = []):
        self.tasks = list(tasks)
        self.constraints = list(constraints)
        self.topics = dict(topics)
        self.initial_control = list(initial_control)

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
                a["signalGetters"] += list(v["signalGetters"])
                # print k, "has", len(a["signalGetters"]), "signals"
            else:
                self.topics[k] = dict(v)
        self.initial_control += other.initial_control
        return self

    def setControlSelection (self, selection):
        for t in self.tasks:
            t.controlSelec.value = selection

    def pushTo (self, sot):
        for t in self.tasks:
            sot.push(t.name)
        if len(self.initial_control)>0:
            from dynamic_graph.sot.core.operator import Mix_of_vector
            ic = Mix_of_vector (sot.name + "_initial_control")
            ic.default.value = tuple ([0,] * sot.getSize())
            for func in self.initial_control:
                func (ic, sot = sot)
            plug (ic.sout, sot.q0)

class Posture(Manifold):
    def __init__ (self, name, sotrobot):
        super(Posture, self).__init__()
        from dynamic_graph.sot.core import Task, FeatureGeneric, GainAdaptive
        from dynamic_graph.sot.core.matrix_util import matrixToTuple
        from numpy import identity

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

        self.tp.setWithDerivative (True)

        # Set the gain of the posture task
        setGain(self.tp.gain,(4.9,0.9,0.01,0.9))
        plug(self.tp.gain.gain, self.tp.controlGain)
        plug(self.tp.error, self.tp.gain.error)
        self.tasks = [ self.tp ]
        self.topics = {
                    name: {
                        "type": "vector",
                        "topic": "/hpp/target/position",
                        "signalGetters": [ self._signalPositionRef ] },
                    "vel_" + name: {
                        "type": "vector",
                        "topic": "/hpp/target/velocity",
                        "signalGetters": [ self._signalVelocityRef ] },
                }

    def _signalPositionRef (self): return self.tp.feature.posture
    def _signalVelocityRef (self): return self.tp.feature.postureDot

## Represents a gripper or a handle
class OpFrame(object):
    def __init__ (self, srdf, model = None):
        self.robotName = srdf["robot"]
        self.name = srdf["name"]
        self.key = self.robotName + "/" + self.name
        self.link = srdf["link"]
        pose = transQuatToSE3 (srdf["position"])
        if srdf.has_key ("joints"):
            assert model is not None
            ## Only for grippers
            self.joints = srdf["joints"]
            self._setupParentJoint (self.link, pose, model)
        else:
            ## Only for handles
            self.pose = pose

    def _setupParentJoint (self, link, pose, model):
        frameid = model.getFrameId (link)
        frame = model.frames[frameid]
        
        self.pose = frame.placement * pose
        self.joint = model.names[frame.parent]

class PreGrasp (Manifold):
    def __init__ (self, gripper, handle, otherGraspOnObject = None):
        super(PreGrasp, self).__init__()
        self.gripper = gripper
        self.handle = handle

    def makeTasks(self, sotrobot, withMeasurementOfObjectPos):
        name = PreGrasp.sep.join(["", "pregrasp", self.gripper.name, self.handle.robotName+"/"+self.handle.name])
        self.graspTask = MetaTaskKine6d (name, sotrobot.dynamic,
                self.gripper.joint, self.gripper.joint)

        setGain(self.graspTask.gain,(4.9,0.9,0.01,0.9))
        self.graspTask.task.setWithDerivative (False)

        # Current gripper position
        # M = se3ToTuple(self.gripper.pose)
        # self.graspTask.opPointModif.activ = True
        # self.graspTask.opPointModif.setTransformation (M)
        self.graspTask.opmodif = se3ToTuple(self.gripper.pose)

        # Express the velocities in local frame.
        # This is the default.
        # self.graspTask.opPointModif.setEndEffector(True)

        # Desired gripper position:
        # Planned handle pose H_p = object_planned_pose * self.handle.pose
        # Real   handle pose H_r = object_real_pose   * self.handle.pose
        # Displacement       M   = H_p^-1 * H_r
        # planned gripper pose G_p= joint_planned_pose * self.gripper.pose
        # The derised position is
        # G*_r = G_p * M = G_p * H^-1 * O_p^-1 * O_r * H
        #                = J_p * G * H^-1 * O_p^-1 * O_r * H
        self.gripper_desired_pose = Multiply_of_matrixHomo (name + "_desired")
        if withMeasurementOfObjectPos:
            self.gripper_desired_pose.setSignalNumber (5)
            # self.gripper_desired_pose.sin0 -> plug to joint planning pose
            self.gripper_desired_pose.sin1.value = se3ToTuple (self.gripper.pose * self.handle.pose.inverse())
            # self.gripper_desired_pose.sin2 -> plug to object planning pose
            # self.gripper_desired_pose.sin3 -> plug to object real pose
            self.gripper_desired_pose.sin4.value = se3ToTuple (self.handle.pose)

            plug(self.gripper_desired_pose.sout, self.graspTask.featureDes.position)
            self.topics = {
                    self.gripper.key: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": self.gripper.robotName + '/' + self.gripper.joint,
                        "signalGetters": [ self._signalJointPlannningPose ] },
                    self.handle.key: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": self.handle.robotName + '/' + self.handle.joint,
                        "signalGetters": [ self._signalObjectPlannningPose ] },
                    self.handle.key: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "tf_listener",
                        "frame0": "world",
                        "frame1": self.handle.robotName + '/' + self.handle.joint,
                        "signalGetters": [ self._signalObjectRealPose ] },
                    }
        else:
            # G*_r = J_p * G
            self.gripper_desired_pose.setSignalNumber (2)
            # self.gripper_desired_pose.sin0 -> plug to joint planning pose
            self.gripper_desired_pose.sin1.value = se3ToTuple (self.gripper.pose)
            self.topics = {
                    # self.gripper.key: {
                    self.gripper.robotName + '/' + self.gripper.joint : {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": self.gripper.robotName + '/' + self.gripper.joint,
                        "signalGetters": [ self._signalJointPlannningPose ] },
                    }

        print self.topics
        # print self.gripper.pose

        plug(self.gripper_desired_pose.sout, self.graspTask.featureDes.position)

        self.graspTask.feature.selec.value = "111111"
        self.tasks = [ self.graspTask.task ]
        # TODO Add velocity

    def _signalJointPlannningPose  (self): return self.gripper_desired_pose.sin0
    def _signalObjectPlannningPose (self): return self.gripper_desired_pose.sin2
    def _signalObjectRealPose      (self): return self.gripper_desired_pose.sin3

class Grasp (Manifold):
    def __init__ (self, gripper, handle, otherGraspOnObject = None, closeGripper = False):
        super(Grasp, self).__init__()
        self.gripper = gripper
        self.handle = handle
        self.closeGripper = closeGripper

        # self.relative = otherGraspOnObject is not None \
                # and otherGraspOnObject.handle.link == self.handle.link
        # TODO: We should make sure that the relative position is constant
        # but we have no way to do it (HPP does). We assume that the object is
        # not articulated.
        self.relative = (otherGraspOnObject is not None)
        if self.relative:
            self.otherGripper = otherGraspOnObject.gripper
            self.otherHandle = otherGraspOnObject.handle

    def makeTasks(self, sotrobot):
        if self.relative:
            self.graspTask = MetaTaskKine6dRel (
                    Grasp.sep + self.gripper.name + Grasp.sep + self.otherGripper.name,
                    sotrobot.dynamic,
                    self.gripper.joint,
                    self.otherGripper.joint,
                    self.gripper.joint,
                    self.otherGripper.joint)

            M = se3ToTuple(self.gripper.pose * self.handle.pose.inverse())
            self.graspTask.opPointModif.activ = True
            self.graspTask.opPointModif.setTransformation (M)
            # Express the velocities in local frame.
            # This is the default.
            # self.graspTask.opPointModif.setEndEffector(True)

            M = se3ToTuple(self.otherGripper.pose * self.otherHandle.pose.inverse())
            self.graspTask.opPointModifBase.activ = True
            self.graspTask.opPointModifBase.setTransformation (M)
            # Express the velocities in local frame.
            # This is the default.
            # self.graspTask.opPointModif.setEndEffector(True)

            # Plug the signals
            plug(self.graspTask.opPointModif    .signal('position'),self.graspTask.feature.position )
            plug(self.graspTask.opPointModif    .signal('jacobian'),self.graspTask.feature.Jq)
            plug(self.graspTask.opPointModifBase.signal('position'),self.graspTask.feature.positionRef )
            plug(self.graspTask.opPointModifBase.signal('jacobian'),self.graspTask.feature.JqRef)
            
            setGain(self.graspTask.gain,(4.9,0.9,0.01,0.9))
            self.graspTask.task.setWithDerivative (False)

            self.tasks = [ self.graspTask.task ]

            # TODO Add velocity
            # self.graspTask.task.setWithDerivative (True)
            # which needs the following signals:
            # - velocity of object attached to gripper.sotjoint, into self.graspTask.feature.dotposition
            # - velocity of object attached to otherGripper.sotjoint, into self.graspTask.feature.dotpositionRef

class EEPosture (Manifold):
    def __init__ (self, sotrobot, gripper, position):
        from dynamic_graph.sot.core import Task, GainAdaptive

        super(EEPosture, self).__init__()
        self.gripper = gripper
        self.jointNames = gripper.joints
        pinmodel = sotrobot.dynamic.model

        n = "eeposture" + Posture.sep + gripper.name + Posture.sep + str(list(position))

        self.tp = Task ('task' + n)
        self.tp.dyn = sotrobot.dynamic
        self.tp.feature = FeaturePosture ('feature_' + n)

        plug(sotrobot.dynamic.position, self.tp.feature.state)
        q = list(sotrobot.dynamic.position.value)
        # Define the reference and the selected DoF
        rank = 0
        ranks = []
        for name in self.jointNames:
            idJ = pinmodel.getJointId(name)
            assert idJ < pinmodel.njoints
            joint = pinmodel.joints[idJ]
            idx_v = joint.idx_v
            nv = joint.nv
            ranks += range(idx_v, idx_v + nv)
            q[idx_v:idx_v+nv] = position[rank: rank + nv]
            rank += nv
        assert rank == len(position)

        self.tp.feature.posture.value = q
        for i in ranks:
            self.tp.feature.selectDof (i, True)

        self.tp.gain = GainAdaptive("gain_"+n)
        self.tp.add(self.tp.feature.name)

        # Set the gain of the posture task
        setGain(self.tp.gain,(4.9,0.9,0.01,0.9))
        plug(self.tp.gain.gain, self.tp.controlGain)
        plug(self.tp.error, self.tp.gain.error)
        if len(self.jointNames) > 0:
            self.tasks = [ self.tp ]

class Foot (Manifold):
    def __init__ (self, footname, sotrobot, selec='111111'):
        robotname, sotjoint = parseHppName (footname)
        self.taskFoot = MetaTaskKine6d(
                Foot.sep + footname,
                sotrobot.dynamic,sotjoint,sotjoint)
        self.taskFoot.feature.selec.value = selec
        super(Foot, self).__init__(
                tasks = [ self.taskFoot.task, ],
                topics = {
                    footname: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": footname,
                        "signalGetters": [ self._signalPositionRef ] },
                    # "vel_" + self.gripper.name: {
                    "vel_" + footname: {
                        "velocity": True,
                        "type": "vector",
                        "handler": "hppjoint",
                        "hppjoint": footname,
                        "signalGetters": [ self._signalVelocityRef ] },
                    })
        self.taskFoot.gain.value = 5

    def _signalPositionRef (self): return self.taskFoot.featureDes.position
    def _signalVelocityRef (self): return self.taskFoot.featureDes.velocity

class COM (Manifold):
    sep = "_com_"
    def __init__ (self, comname, sotrobot):
        self.taskCom = MetaTaskKineCom (sotrobot.dynamic,
                name = COM.sep + comname)
        self.taskCom.task.setWithDerivative (True)
        super(COM, self).__init__(
                tasks = [ self.taskCom.task, ],
                topics = {
                    comname: {
                        "velocity": False,
                        "type": "vector3",
                        "handler": "hppcom",
                        "hppcom": comname,
                        "signalGetters": [ self._signalPositionRef ] },
                    "vel_" + comname: {
                        "velocity": True,
                        "type": "vector3",
                        "handler": "hppcom",
                        "hppcom": comname,
                        "signalGetters": [ self._signalVelocityRef ] },
                    })
        self.taskCom.task.controlGain.value = 5

    def _signalPositionRef (self): return self.taskCom.featureDes.errorIN
    def _signalVelocityRef (self): return self.taskCom.featureDes.errordotIN
