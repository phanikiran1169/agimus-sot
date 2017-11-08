from hpp import Transform
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.core.meta_tasks_kine_relative import MetaTaskKine6dRel
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture

def parseHppName (hppjointname):
    return hppjointname.split('/', 1)

def transformToMatrix (T):
    from numpy import eye
    M = eye (4)
    M[:3,:3] = T.quaternion.toRotationMatrix()
    M[:3,3] = T.translation
    return M

class Manifold(object):
    sep = "___"

    def __init__ (self, tasks = [], constraints = [], topics = {}):
        self.tasks = tasks
        self.constraints = constraints
        self.topics = topics

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
        from dynamic_graph.sot.core import Task, FeatureGeneric, GainAdaptive, Selec_of_vector
        from dynamic_graph.sot.core.meta_tasks import setGain
        from dynamic_graph.sot.core.matrix_util import matrixToTuple
        from dynamic_graph import plug
        from numpy import identity, hstack, zeros

        n = Posture.sep + name
        self.tp = Task ('task' + n)
        self.tp.dyn = sotrobot.dynamic
        self.tp.feature = FeatureGeneric('feature_'+n)
        self.tp.featureDes = FeatureGeneric('feature_des_'+n)
        self.tp.gain = GainAdaptive("gain_"+n)
        robotDim = sotrobot.dynamic.getDimension()
        first_6 = zeros((robotDim-6,6))
        other_dof = identity(robotDim-6)
        jacobian_posture = hstack([first_6, other_dof])
        self.tp.feature.jacobianIN.value = matrixToTuple( jacobian_posture )
        self.tp.feature.setReference(self.tp.featureDes.name)
        self.tp.add(self.tp.feature.name)

        # Connects the dynamics to the current feature of the posture task
        # plug(re.position, taskPosture.featureDes.errorIN)
        # plug(re.velocity, taskPosture.featureDes.errordotIN)

        getPostureValue = Selec_of_vector("current_posture")
        getVelocityValue = Selec_of_vector("current_velovity")
        getPostureValue.selec(6,robotDim)
        getVelocityValue.selec(6,robotDim)

        plug(sotrobot.dynamic.position, getPostureValue.sin)
        plug(getPostureValue.sout, self.tp.feature.errorIN)
        plug(sotrobot.dynamic.velocity, getVelocityValue.sin)
        plug(getVelocityValue.sout, self.tp.feature.errordotIN)

        # Set the gain of the posture task
        setGain(self.tp.gain,(4.9,0.9,0.01,0.9))
        # setGain(taskPosture.gain,(9.8,1.8,0.02,1.8))
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
    def __init__ (self, gripper, handle, otherGraspOnObject = None):
        super(Grasp, self).__init__()
        self.gripper = gripper
        self.handle = handle
        self.otherGrasp = otherGraspOnObject
        self.relative = self.otherGrasp is not None \
                and self.otherGrasp.handle.hppjoint == self.handle.hppjoint
        if self.relative:
            self.topics = dict()
        else:
            self.topics = {
                    # self.gripper.name: {
                    self.gripper.hppjoint: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": self.gripper.hppjoint,
                        "signalGetters": [ self._signalPositionRef ] },
                    # "vel_" + self.gripper.name: {
                    "vel_" + self.gripper.hppjoint: {
                        "velocity": True,
                        "type": "vector",
                        "handler": "hppjoint",
                        "hppjoint": self.gripper.hppjoint,
                        "signalGetters": [ self._signalVelocityRef ] },
                    }

    def makeTasks(self, sotrobot):
        from dynamic_graph.sot.core.matrix_util import matrixToTuple
        from dynamic_graph.sot.core import Multiply_of_matrixHomo, OpPointModifier
        if self.relative:
            # We define a MetaTaskKine6dRel
            self.graspTask = MetaTaskKine6dRel (
                    Grasp.sep + self.gripper.name + Grasp.sep + self.handle.name +
                    '(rel_to_' + self.otherGrasp.gripper.name + ')',
                    sotrobot.dynamic,
                    self.gripper.sotjoint,
                    self.gripper.sotjoint,
                    self.otherGrasp.gripper.sotjoint,
                    self.otherGrasp.gripper.sotjoint)

            M = transformToMatrix(self.gripper.sotpose * self.handle.sotpose.inverse())
            self.graspTask.opmodif = matrixToTuple(M)
            M = transformToMatrix(self.otherGrasp.handle.sotpose * self.otherGrasp.gripper.sotpose.inverse())
            self.graspTask.opmodifBase = matrixToTuple(M)
        else:
            # We define a MetaTaskKine6d
            self.graspTask = MetaTaskKine6d (
                    self.gripper.name + Grasp.sep + self.handle.name,
                    sotrobot.dynamic,
                    self.gripper.sotjoint,
                    self.gripper.sotjoint)
            # TODO At the moment, the reference is the joint frame, not the gripper frame.
            # M = transformToMatrix(self.gripper.sotpose)
            # self.graspTask.opmodif = matrixToTuple(M)
        # self.graspTask.feature.frame("desired")
        self.graspTask.task.controlGain.value = 5
        self.graspTask.feature.frame("current")
        self.tasks = [ self.graspTask.task ]

    def _signalPositionRef (self): return self.graspTask.featureDes.position
    def _signalVelocityRef (self): return self.graspTask.featureDes.velocity

class Foot (Manifold):
    def __init__ (self, footname, sotrobot):
        robotname, sotjoint = parseHppName (footname)
        self.taskFoot = MetaTaskKine6d(
                Foot.sep + footname,
                sotrobot.dynamic,sotjoint,sotjoint)
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

    def _signalPositionRef (self): return self.taskFoot.featureDes.position
    def _signalVelocityRef (self): return self.taskFoot.featureDes.velocity

class COM (Manifold):
    def __init__ (self, comname, sotrobot):
        self.taskCom = MetaTaskKineCom (sotrobot.dynamic,
                name = COM.sep + comname)
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
