from hpp import Transform
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph.sot.core.meta_tasks_kine_relative import MetaTaskKine6dRel
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core import FeaturePosture

def parseHppName (hppjointname):
    if hppjointname == "universe": return "", "universe"
    return hppjointname.split('/', 1)

def transformToTuple (T):
    from dynamic_graph.sot.core.matrix_util import matrixToTuple
    from numpy import eye
    M = eye (4)
    M[:3,:3] = T.quaternion.toRotationMatrix()
    M[:3,3] = T.translation
    return matrixToTuple(M)

class Manifold(object):
    sep = "___"

    def __init__ (self, tasks = [], constraints = [], topics = {}):
        self.tasks = list(tasks)
        self.constraints = list(constraints)
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
                a["signalGetters"] += list(v["signalGetters"])
                # print k, "has", len(a["signalGetters"]), "signals"
            else:
                self.topics[k] = dict(v)
        return self

    def pushTo (self, sot):
        for t in self.tasks:
            sot.push(t.name)

class Posture(Manifold):
    def __init__ (self, name, sotrobot):
        super(Posture, self).__init__()
        from dynamic_graph.sot.core import Task, FeatureGeneric, GainAdaptive
        from dynamic_graph.sot.core.matrix_util import matrixToTuple
        from dynamic_graph import plug
        from numpy import identity

        n = Posture.sep + name
        self.tp = Task ('task' + n)
        self.tp.dyn = sotrobot.dynamic
        self.tp.feature = FeatureGeneric('feature_'+n)
        self.tp.featureDes = FeatureGeneric('feature_des_'+n)
        self.tp.gain = GainAdaptive("gain_"+n)
        robotDim = sotrobot.dynamic.getDimension()
        self.tp.feature.jacobianIN.value = matrixToTuple( identity(robotDim) )
        self.tp.feature.setReference(self.tp.featureDes.name)
        self.tp.add(self.tp.feature.name)

        # Connects the dynamics to the current feature of the posture task
        plug(sotrobot.dynamic.position, self.tp.feature.errorIN)

        self.tp.setWithDerivative (True)

        # Set the gain of the posture task
        setGain(self.tp.gain,10)
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

    def _signalPositionRef (self): return self.tp.featureDes.errorIN
    def _signalVelocityRef (self): return self.tp.featureDes.errordotIN

class OpFrame(object):
    def __init__ (self, hppclient):
        self.hpp = hppclient

    def setHppGripper (self, name):
        self.name = name
        # Get parent joint and position from HPP
        self.hppjoint, self.hpppose = self.hpp.manipulation.robot.getGripperPositionInJoint(name)
        self.hpppose = Transform (self.hpppose)

    def setHppHandle (self, name):
        self.name = name
        # Get parent joint and position from HPP
        self.hppjoint, self.hpppose = self.hpp.manipulation.robot.getHandlePositionInJoint(name)
        self.hpppose = Transform (self.hpppose)

    def setSotFrameFromHpp(self, pinrobot):
        # The joint should be available in the robot model used by SOT
        n = self.hppjoint
        self.sotpose = self.hpppose
        self.robotname, self.sotjoint = parseHppName (n)
        while self.sotjoint not in pinrobot.names:
            self.sotpose = Transform(self.hpp.basic.robot.getJointPositionInParentFrame(n)) * self.sotpose
            n = self.hpp.basic.robot.getParentJointName(n)
            robotname, self.sotjoint = parseHppName (n)

class Grasp (Manifold):
    def __init__ (self, gripper, handle, otherGraspOnObject = None, closeGripper = False):
        super(Grasp, self).__init__()
        self.gripper = gripper
        self.handle = handle
        self.closeGripper = closeGripper

        self.relative = otherGraspOnObject is not None \
                and otherGraspOnObject.handle.hppjoint == self.handle.hppjoint
        if self.relative:
            self.otherGripper = otherGraspOnObject.gripper
            self.otherHandle = otherGraspOnObject.handle

    def makeTasks(self, sotrobot):
        from dynamic_graph import plug
        if self.relative:
            self.graspTask = MetaTaskKine6dRel (
                    Grasp.sep + self.gripper.name + Grasp.sep + self.otherGripper.name,
                    sotrobot.dynamic,
                    self.gripper.sotjoint,
                    self.otherGripper.sotjoint,
                    self.gripper.sotjoint,
                    self.otherGripper.sotjoint)

            M = transformToTuple(self.gripper.sotpose * self.handle.sotpose.inverse())
            self.graspTask.opPointModif.activ = True
            self.graspTask.opPointModif.setTransformation (M)
            # Express the velocities in local frame.
            # This is the default.
            # self.graspTask.opPointModif.setEndEffector(True)

            M = transformToTuple(self.otherGripper.sotpose * self.otherHandle.sotpose.inverse())
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
        from dynamic_graph import plug

        super(EEPosture, self).__init__()
        self.gripper = gripper
        robotName, gripperName = parseHppName (gripper.name)
        self.jointNames = sotrobot.grippers[gripperName]
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
