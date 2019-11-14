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

from hpp.corbaserver.manipulation.constraint_graph_factory import ConstraintFactoryAbstract, GraphFactoryAbstract
from tools import Manifold, Grasp, PreGrasp, OpFrame, EndEffector
from .solver import Solver

## Affordance between a gripper and a handle.
#
# This class allows to tune the behaviour of the robot when grasping of
# releasing an object. There are several behaviour already implemented.
class Affordance(object):
    ## Constructor
    # \param gripper names of the gripper
    # \param handle  names of the handle
    # \param openControlType, closeControlType control type for releasing and
    #        grasping the object. Must be one of \c "position", \c "torque" or
    #        \c "position_torque"
    # \param refs a dictionnary of reference values.
    #             Keys should be \c "angle_open", \c "angle_close" and \c "torque")
    # \param controlParams parameters of the control:
    # \param simuParams parameters of the torque feedback simulation.
    #
    # Control parameters depend on the control type.
    # \li For \c "position", no parameters.
    # \li For \c "torque", see control.AdmittanceControl constructor (torque_num, torque_denom)
    # \li For \c "position_torque", see control.PositionAndAdmittanceControl constructor.
    # (wn, z, torque_num, torque_denom)
    def __init__ (self, gripper, handle, openControlType, closeControlType,
            refs, controlParams = {}, simuParams = {}):
        self.gripper = gripper
        self.handle  = handle
        self.controlType = { "open": openControlType, "close": closeControlType, }
        self.ref = refs
        self.controlParams = controlParams
        self.simuParams = simuParams

    ## Very likely never used but who knows...
    # \todo remove me
    def setControl (self, refOpen, refClose, openType = "position", closeType="position"):
        from warnings import warn
        warn("Method Affordance.getControl will be deleted soon!")
        if openType != "position" or closeType != "position":
            raise NotImplementedError ("Only position control is implemented for gripper opening/closure.")
        self.controlType = {
                "open": openType,
                "close": closeType,
                }
        self.ref = {
                "angle_open": refOpen,
                "angle_close": refClose,
                }

    ## Get position control parameter
    def getControlParameter (self):
        wn    = self.controlParams.get("wn",    10.)
        z     = self.controlParams.get("z",     1. )
        nums_tor   = self.controlParams.get("torque_num", (1.,))
        denoms_tor = self.controlParams.get("torque_denom", (1.,))
        return wn, z, nums_tor, denoms_tor

    ## Simulation parameters
    # \li mass    defaults to 0.
    # \li damping defaults to 5.
    # \li spring  defaults to 100.
    # \li refPos  defaults to \c self.ref["angle_close"]
    def getSimulationParameters (self):
        mass    = self.simuParams.get("mass"   , 0.)
        damping = self.simuParams.get("damping", 5. )
        spring  = self.simuParams.get("spring" , 100. )
        refPos  = self.simuParams.get("refPos" , self.ref["angle_close"] )
        return mass,damping,spring,refPos

    ## \todo Provide a way of fine granularity visual feedback specification.
    #        For instance, via an attribute disableGripperVisualFeedback
    def useMeasurementOfGripperPose (self, gripperFrame):
        return gripperFrame.hasVisualTag

    ## \todo Provide a way of fine granularity visual feedback specification.
    #        For instance, via an attribute disableObjectVisualFeedback
    def useMeasurementOfObjectPose (self, handleFrame):
        return handleFrame.hasVisualTag

## Create \ref tools.Manifold s
# 
# \sa manipulation.constraint_graph_factory.ConstraintFactoryAbstract
class TaskFactory(ConstraintFactoryAbstract):
    gfields = ('grasp', 'pregrasp', 'gripper_open', 'gripper_close')
    pfields = ()

    def __init__ (self, graphfactory):
        super(TaskFactory, self).__init__ (graphfactory)
        self._grippers = dict()

    def _buildGripper (self, type, gripper, handle):
        key = (type, gripper, handle)
        if self._grippers.has_key (key):
            return self._grippers[key]
        try:
            aff = self.graphfactory.affordances[(gripper, handle)]
        except KeyError:
            # If there are no affordance, do not add a task.
            self._grippers[key] = Manifold ()
            return self._grippers[key]
        gf = self.graphfactory
        gripperFrame = gf.gripperFrames[gripper]
        if not gripperFrame.enabled:
            self._grippers[key] = Manifold ()
            return self._grippers[key]
        robot = gf.sotrobot
        if aff.controlType[type] == "position":
            ee = EndEffector (robot, gripperFrame, "p" + type + ("_" + handle if handle is not None else ""))
            ee.makePositionControl (aff.ref["angle_"+type])
            self._grippers[key] = ee
        elif aff.controlType[type] == "torque" or aff.controlType[type] == "position_torque":
            ee = EndEffector (robot, gripperFrame, "pt_" + type + ("_" + handle if handle is not None else ""))
            ee.makeAdmittanceControl (aff, type,
                    period = gf.parameters["period"],
                    simulateTorqueFeedback = gf.parameters.get("simulateTorqueFeedback",False))
            if gf.parameters["addTracerToAdmittanceController"]:
                tracer = ee.ac.addTracerRealTime (robot)
                gf.tracers[tracer.name] = tracer
            self._grippers[key] = ee
        else:
            raise NotImplementedError ("Control type " + type + " is not implemented for gripper.")
        return self._grippers[key]

    def buildGrasp (self, g, h, otherGrasp=None):
        gf = self.graphfactory
        if h is None:
            gripper_open = self._buildGripper ("open", g, h)
            return { 'gripper_open': gripper_open }
        gripper = gf.gripperFrames[g]
        handle  = gf.handleFrames [h]

        # get affordance
        try:
            aff = gf.affordances[(g, h)]
            useMeasurementOfObjectPose  = aff. useMeasurementOfObjectPose(handle)
            useMeasurementOfGripperPose = aff.useMeasurementOfGripperPose(gripper)
        except KeyError:
            useMeasurementOfObjectPose  = handle .hasVisualTag
            useMeasurementOfGripperPose = gripper.hasVisualTag

        gripper_close = self._buildGripper ("close", g, h)
        pregrasp = PreGrasp (gripper, handle, otherGrasp)
        pregrasp.makeTasks (gf.sotrobot,
                useMeasurementOfObjectPose,
                useMeasurementOfGripperPose)

        if not gripper.enabled:
            # TODO If otherGrasp is not None,
            # we should include the grasp function of otherGrasp, not pregrasp function...
            grasp = Manifold()
        else:
            grasp = Grasp (gripper, handle, otherGrasp)
            grasp.makeTasks (gf.sotrobot)
        return { 'grasp': grasp,
                 'pregrasp': pregrasp,
                 'gripper_close': gripper_close }

    ## \name Accessors to the different elementary constraints
    # \{
    def getGrasp(self, gripper, handle, otherGrasp=None):
        if isinstance(gripper, str): ig = self.graphfactory.grippers.index(gripper)
        else: ig = gripper
        if isinstance(handle, str): ih = self.graphfactory.handles.index(handle)
        else: ih = handle
        if otherGrasp is not None:
            otherIg = self.graphfactory.grippers.index(otherGrasp[0].key)
            otherIh = self.graphfactory.handles.index(otherGrasp[1].key)
            k = (ig, ih, otherIg, otherIh)
        else:
            k = (ig, ih)
        if not self._grasp.has_key(k):
            self._grasp[k] = self.buildGrasp(self.graphfactory.grippers[ig], None if ih is None else self.graphfactory.handles[ih], otherGrasp)
            assert isinstance (self._grasp[k], dict)
        return self._grasp[k]

    def g (self, gripper, handle, what, otherGrasp=None):
        return self.getGrasp(gripper, handle, otherGrasp)[what]
    # \}

    def event (self, gripper, handle, what, default):
        if handle is None:
            ee = self._buildGripper ("open", gripper, handle)
        else:
            ee = self._buildGripper ("close", gripper, handle)
        return ee.events.get(what, default)

    def buildPlacement (self, o):
        # Nothing to do
        return dict()

## Create a set of controllers for a set of tasks.
#
# A controller is created for each transition of the graph of constraints.
#
# See the following example for usage.
# \code{.py}
# from agimus_sot import Supervisor
# from agimus_sot.factory import Factory, Affordance
# from agimus_sot.tools import Manifold
# from agimus_sot.srdf_parser import parse_srdf
# from hpp.corbaserver.manipulation import Rule
#
# # Constraint graph definition. Should be the same as the one used for planning
# # in HPP.
# grippers = [ "talos/left_gripper", ]
# objects = [ "box" ]
# handlesPerObjects = [ [ "box/handle1", "box/handle2" ], ]
# contactPerObjects = [ [ "box/bottom_surface", ] ]
# rules = [
#           Rule([ "talos/left_gripper", ], [ "box/handle2", ], False),
#           # Rule([ "talos/left_gripper", ], [ Object.handles[0], ], True),
#           Rule([ "talos/left_gripper", ], [ ".*", ], True),
#           # Rule([ "talos/right_gripper", ], [ Object.handles[1], ], True),
#           ]
#
# # Parse SRDF files to extract gripper and handle information.
# srdf = {}
# srdfTalos = parse_srdf ("srdf/talos.srdf", packageName = "talos_data", prefix="talos")
# srdfBox   = parse_srdf ("srdf/cobblestone.srdf", packageName = "gerard_bauzil", prefix="box")
# srdfTable = parse_srdf ("srdf/pedestal_table.srdf", packageName = "gerard_bauzil", prefix="table")
# for w in [ "grippers", "handles" ]:
#     srdf[w] = dict()
#     for d in [ srdfTalos, srdfBox, srdfTable ]:
#         srdf[w].update (d[w])
#
#
# supervisor = Supervisor (robot, hpTasks = hpTasks(robot))
# factory = Factory(supervisor)
#
# # Define parameters
# factory.parameters["period"] = robot.getTimeStep() # This must be made available for your robot
# factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
# factory.parameters["addTracerToAdmittanceController"] = True
#
# factory.setGrippers (grippers)
# factory.setObjects (objects, handlesPerObjects, contactPerObjects)
# factory.environmentContacts (["table/support",])
# factory.setRules (rules)
# factory.setupFrames (srdf["grippers"], srdf["handles"], robot, disabledGrippers=["table/pose",])
# # At the moment, one must manually enable visual feedback
# factory.gripperFrames["talos/left_gripper"].hasVisualTag = True
# factory.handleFrames["box/handle1"].hasVisualTag = True
# factory.addAffordance (
#     Affordance ("talos/left_gripper", "box/handle1",
#         openControlType="torque", closeControlType="torque",
#         refs = { "angle_open": (0,), "angle_close": (-0.5,), "torque": (-0.05,) },
#         controlParams = { "torque_num": ( 5000., 1000.),
#             "torque_denom": (0.01,) },
#         simuParams = { "refPos": (-0.2,) }))
# factory.addAffordance (
#     Affordance ("talos/left_gripper", None,
#         openControlType="position", closeControlType="position",
#         refs = { "angle_open": (0,), "angle_close": (-0.5,), "torque": (-0.05,) },
#         simuParams = { "refPos": (-0.2,) }))
# factory.generate ()
#
# supervisor.makeInitialSot ()
# \endcode
# 
# \sa manipulation.constraint_graph_factory.GraphFactoryAbstract, TaskFactory,
#     Affordance
class Factory(GraphFactoryAbstract):
    class State:
        def __init__ (self, tasks, grasps, factory):
            self.name = factory._stateName (grasps)
            self.grasps = grasps
            self.manifold = Manifold()

            self.objectsAlreadyGrasped = {}
            
            for ig, ih in enumerate(grasps):
                if ih is not None:
                    # Add task gripper_close
                    self.manifold += tasks.g (factory.grippers[ig], factory.handles[ih], 'gripper_close')
                    otherGrasp = self.objectsAlreadyGrasped.get(factory.objectFromHandle[ih])
                    self.manifold += tasks.g (factory.grippers[ig], factory.handles[ih], 'grasp', otherGrasp)
                    self.objectsAlreadyGrasped[factory.objectFromHandle[ih]] = (
                            factory.gripperFrames[factory.grippers[ig]],
                            factory. handleFrames[factory.handles[ih]],)
                else:
                    # Add task gripper_open
                    self.manifold += tasks.g (factory.grippers[ig], None, 'gripper_open')

    def __init__ (self, supervisor):
        super(Factory, self).__init__ ()
        self.tasks = TaskFactory (self)
        self.hpTasks = supervisor.hpTasks
        self.lpTasks = supervisor.lpTasks
        self.affordances = dict()
        self.sots = dict()
        ## A dictionnary
        # - key: name of the transition after which an action must be done
        # - value: dictionnary:
        #          - key: the reached state (at most two values, depending on whether the dst state was reached)
        #          - value: sot representing the post-action
        self.postActions = dict()
        ## A dictionnary
        # - key: name of the transition before which an action must be done
        # - value: sot representing the pre-action
        self.preActions = dict()

        self.tracers = {}
        self.controllers = {}
        self.supervisor = supervisor

        ## Accepted parameters:
        ## - period: [double, no default]
        ##           Time interval between two iterations of the graph.
        ## - simulateTorqueFeedback: [boolean, False]
        ##                           do not use torque feedback from the robot
        ##                           but simulate it instead.
        self.parameters = {
                "addTracerToAdmittanceController": False,
                "addTimerToSotControl": False,
                "addTracerToSotControl": False,
                "simulateTorqueFeedback": False,
                }

    def _newSoT (self, name):
        # Create a solver
        sot = Solver (name,
                self.sotrobot.dynamic.getDimension(),
                damping = 0.001,
                timer = self.parameters["addTimerToSotControl"],
                )
        # Make default event signals
        # sot. doneSignal = self.supervisor.controlNormConditionSignal()
        from .events import logical_and_entity
        sot. doneSignal = logical_and_entity ("ade_sot_"+sot.name,
                [ self.supervisor.controlNormConditionSignal(),
                  self.supervisor. done_events.timeEllapsedSignal])
        sot.errorSignal = False

        if self.parameters["addTimerToSotControl"]:
            id = len(self.SoTtracer.signals()) - 1
            self.SoTtracer.add (sot.timer.name + ".timer", "solver_"+str(id) + ".timer")
        if self.parameters["addTracerToSotControl"]:
            id = len(self.SoTtracer.signals()) - 1
            self.SoTtracer.add (sot.controlname, "solver_"+str(id) + ".control")
        return sot

    def addAffordance (self, aff, simulateTorqueFeedback = False):
        assert isinstance(aff, Affordance)
        self.affordances [(aff.gripper, aff.handle)] = aff

    def generate (self):
        if self.parameters["addTimerToSotControl"] or self.parameters["addTracerToSotControl"]:
            # init tracer
            from dynamic_graph.tracer_real_time import TracerRealTime
            SoTtracer = TracerRealTime ("tracer_of_solvers")
            self.tracers[SoTtracer.name] = SoTtracer
            SoTtracer.setBufferSize (10 * 1048576) # 10 Mo
            SoTtracer.open ("/tmp", "sot-control-trace", ".txt")
            self.sotrobot.device.after.addSignal("tracer_of_solvers.triger")
            self.supervisor.SoTtracer = SoTtracer
            self.SoTtracer = SoTtracer
        super(Factory, self).generate ()

        self.supervisor.sots = {}
        self.supervisor.grasps = { (gh, w): t for gh, ts in self.tasks._grasp.items() for w, t in ts.items() }
        self.supervisor.hpTasks = self.hpTasks
        self.supervisor.lpTasks = self.lpTasks
        self.supervisor.postActions = {}
        self.supervisor.preActions  = {}
        self.supervisor.tracers = self.tracers
        self.supervisor.controllers = self.controllers

        from dynamic_graph import plug
        self.supervisor.sots_indexes = dict()
        for tn,sot in self.sots.iteritems():
            # Pre action
            if self.preActions.has_key(tn):
                self.supervisor.addPreAction (tn, self.preActions[tn])
            # Action
            self.supervisor.addSolver (tn, sot)
            # Post action
            if self.postActions.has_key(tn):
                self.supervisor.addPostActions (tn, self.postActions[tn])

    def setupFrames (self, srdfGrippers, srdfHandles, sotrobot, disabledGrippers = ()):
        self.sotrobot = sotrobot

        self.grippersIdx = { g: i for i,g in enumerate(self.grippers) }
        self.handlesIdx  = { h: i for i,h in enumerate(self.handles) }

        self.gripperFrames = { g: OpFrame(srdfGrippers[g], sotrobot.dynamic.model, g not in disabledGrippers) for g in self.grippers }
        self.handleFrames  = { h: OpFrame(srdfHandles [h]                                                   ) for h in self.handles  }

        # Compute the DoF which should not be affected by the
        # task not related to end-effectors.
        gripper_joints = []
        for g,gripper in self.gripperFrames.iteritems():
            try:
                gripper_joints += gripper.joints
            except:
                print "Gripper", gripper.name, "has not joints. Cannot be controlled."
        from .tools import computeControlSelection
        # print "Remove gripper_joints from solvers", gripper_joints
        self.dof_selection = computeControlSelection (sotrobot,gripper_joints)
        self.hpTasks.setControlSelection (self.dof_selection)
        self.lpTasks.setControlSelection (self.dof_selection)

    def makeState (self, grasps, priority):
        # Nothing to do here
        return Factory.State(self.tasks, grasps, self)

    def makeLoopTransition (self, state):
        n = self._loopTransitionName(state.grasps)
        sot = self._newSoT ('sot_'+n)
        from .events import logical_and_entity
        sot. doneSignal = logical_and_entity("ade_sot_"+n,
                [   self.supervisor. done_events.timeEllapsedSignal,
                    self.supervisor.controlNormConditionSignal() ])

        self.hpTasks.pushTo(sot)
        state.manifold.pushTo(sot)
        self.lpTasks.pushTo(sot)

        self.sots[n] = sot

    def makeTransition (self, stateFrom, stateTo, ig):
        sf = stateFrom
        st = stateTo
        names = self._transitionNames(sf, st, ig)

        iobj = self.objectFromHandle [st.grasps[ig]]
        obj = self.objects[iobj]
        noPlace = self._isObjectGrasped (sf.grasps, iobj)
        #TODO compute other grasp on iobj
        # it must be a grasp or pregrasp task
        otherGrasp = sf.objectsAlreadyGrasped.get(iobj ,None)

        # The different cases:
        pregrasp = True
        intersec = not noPlace
        preplace = not noPlace

        # Start here
        nWaypoints = pregrasp + intersec + preplace
        nTransitions = 1 + nWaypoints

        # Link waypoints
        transitions = names[:]
        assert nWaypoints > 0
        M = 1 + pregrasp
        sots = [ ]
        for i in range(nTransitions):
            ns = ("{0}_{1}{2}".format(names[0], i, i+1),
                  "{0}_{2}{1}".format(names[1], i, i+1))

            for n in ns:
                s = self._newSoT('sot_'+n)
                from .events import logical_and_entity
                s. doneSignal = logical_and_entity("ade_sot_"+n,
                        [   self.supervisor. done_events.timeEllapsedSignal,
                            self.supervisor.controlNormConditionSignal() ])

                self.hpTasks.pushTo(s)

                if pregrasp and i == 1:
                    # Add pregrasp task
                    pregraspT = self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'pregrasp', otherGrasp = otherGrasp)
                    pregraspT.pushTo (s)
                if i < M: sf.manifold.pushTo(s)
                else:     st.manifold.pushTo(s)

                self.lpTasks.pushTo(s)
                self.sots[n] = s
                sots.append (n)

        ## Post-actions for transitions from
        # 1. pregrasp to intersec, intersec (st) reached:
        #   - "pregrasp"
        #   - "gripper_close"
        key = sots[2*(M-1)]
        sot = self._newSoT ("postAction_" + key)
        self.hpTasks.pushTo (sot)
        # "gripper_close" is in st.manifold
        # self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'gripper_close').pushTo (sot)
        self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'pregrasp', otherGrasp).pushTo (sot)
        st.manifold.pushTo (sot)
        self.lpTasks.pushTo (sot)
        from .events import logical_and_entity
        sot. doneSignal = logical_and_entity ("ade_sot_"+sot.name,
                [ self.tasks.event (self.grippers[ig], self.handles[st.grasps[ig]],
                    'done_close',
                    self.supervisor.controlNormConditionSignal()),
                    self.supervisor. done_events.timeEllapsedSignal])
        #TODO add error_events "gripper_closed_failed"
        if not self.postActions.has_key(key):
            self.postActions[ key ] = dict()
        self.postActions[ key ] [ st.name ] = sot

        # 2. intersec to pregrasp, pregrasp (sf) reached:
        # TODO Should this post-action be done ?
        # Force the re-alignment with planning.
        key = sots[2*(M-1)+1]
        sot = self._newSoT ("postAction_" + key)
        # TODO Any events ?
        self.hpTasks.pushTo (sot)
        sf.manifold.pushTo (sot)
        self.lpTasks.pushTo (sot)
        if not self.postActions.has_key(key):
            self.postActions[ key ] = dict()
        self.postActions[ key ] [ sf.name ] = sot


        ## Pre-actions for transitions from

        # 1.intersec to pregrasp:
        #   - "pregrasp": the motion must be relative to the object
        #   - "gripper_open"
        key = sots[2*(M-1) + 1]
        sot = self._newSoT ("preAction_" + key)
        self.hpTasks.pushTo (sot)
        # "gripper_open" is in sf.manifold
        # self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'gripper_open').pushTo (sot)
        self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'pregrasp', otherGrasp).pushTo (sot)
        sf.manifold.pushTo (sot)
        self.lpTasks.pushTo (sot)
        # sot. doneSignal = self.tasks.event (self.grippers[ig], self.handles[st.grasps[ig]],
                # 'done_open', self.supervisor.controlNormConditionSignal())

        sot. doneSignal = logical_and_entity ("ade_sot_"+sot.name,
                [ self.tasks.event (self.grippers[ig], None,
                    'done_open',
                    self.supervisor.controlNormConditionSignal()),
                    self.supervisor. done_events.timeEllapsedSignal])
        #TODO add error_events "gripper_open_failed"
        self.preActions[ key ] = sot

        # 2. pregrasp to intersec:
        #   - "pregrasp": the motion must be relative to the object
        # Required to force the alignment gripper / handle before the actual grasp.
        key = sots[2*(M-1)]
        sot = self._newSoT ("preAction_" + key)
        self.hpTasks.pushTo (sot)
        self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'pregrasp', otherGrasp).pushTo (sot)
        sf.manifold.pushTo (sot)
        self.lpTasks.pushTo (sot)
        # Default events should be fine.
        self.preActions[ key ] = sot
