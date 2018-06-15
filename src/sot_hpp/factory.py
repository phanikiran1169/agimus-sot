from hpp.corbaserver.manipulation.constraint_graph_factory import ConstraintFactoryAbstract, GraphFactoryAbstract
from tools import Manifold, Grasp, OpFrame, EEPosture
from dynamic_graph.sot.core import SOT

class Affordance(object):
    def __init__ (self, gripper, handle, **kwargs):
        self.gripper = gripper
        self.handle  = handle
        self.setControl (**kwargs)
    def setControl (self, refOpen, refClose, openType = "position", closeType="position"):
        if openType != "position" or closeType != "position":
            raise NotImplementedError ("Only position control is implemented for gripper opening/closure.")
        self.controlType = {
                "open": openType,
                "close": closeType,
                }
        self.ref = {
                "open": refOpen,
                "close": refClose,
                }

class TaskFactory(ConstraintFactoryAbstract):
    gfields = ('grasp', 'pregrasp', 'gripper_open', 'gripper_close')
    pfields = ()

    def __init__ (self, graphfactory):
        super(TaskFactory, self).__init__ (graphfactory)

    def _buildGripper (self, type, gripper, handle):
        try:
            aff = self.graphfactory.affordances[(gripper, handle)]
        except KeyError:
            # If there are no affordance, do not add a task.
            return Manifold()
        gf = self.graphfactory
        robot = gf.sotrobot
        if aff.controlType[type] == "position":
            return EEPosture (robot, gf.gripperFrames[gripper], aff.ref[type])
        else:
            raise NotImplementedError ("Only position control is implemented for gripper closure.")

    def buildGrasp (self, g, h, otherGrasp=None):
        gf = self.graphfactory
        if h is None:
            gripper_open = self._buildGripper ("open", g, h)
            return { 'gripper_open': gripper_open }

        gripper_close  = self._buildGripper ("close", g, h)
        pregrasp = Grasp (gf.gripperFrames [g],
                          gf.handleFrames [h],
                          otherGrasp,
                          False)
        pregrasp.makeTasks (gf.sotrobot)
        grasp = Grasp (gf.gripperFrames [g],
                       gf.handleFrames [h],
                       otherGrasp,
                       True)
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
            otherIg = self.graphfactory.grippers.index(otherGrasp.gripper.key)
            otherIh = self.graphfactory.handles.index(otherGrasp.handle.key)
            k = (ig, ih, otherIg, otherIh)
        else:
            k = (ig, ih)
        if not self._grasp.has_key(k):
            self._grasp[k] = self.buildGrasp(self.graphfactory.grippers[ig], None if ih is None else self.graphfactory.handles[ih], otherGrasp)
            assert isinstance (self._grasp[k], dict)
        return self._grasp[k]

    def g (self, gripper, handle, what, otherGrasp=None):
        return self.getGrasp(gripper, handle, otherGrasp)[what]

    def buildPlacement (self, o):
        # Nothing to do
        return dict()

class Factory(GraphFactoryAbstract):
    class State:
        def __init__ (self, tasks, grasps, factory):
            self.name = factory._stateName (grasps)
            self.grasps = grasps
            self.manifold = Manifold()

            objectsAlreadyGrasped = {}
            
            for ig, ih in enumerate(grasps):
                if ih is not None:
                    # Add task gripper_close
                    self.manifold += tasks.g (factory.grippers[ig], factory.handles[ih], 'gripper_close')
                    otherGrasp = objectsAlreadyGrasped.get(factory.objectFromHandle[ih])
                    self.manifold += tasks.g (factory.grippers[ig], factory.handles[ih], 'grasp', otherGrasp)
                    objectsAlreadyGrasped[factory.objectFromHandle[ih]] = tasks.g (factory.grippers[ig], factory.handles[ih], 'grasp', otherGrasp)
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
        self.postActions = dict()
        self.preActions = dict()

        self.timers = {}
        self.supervisor = supervisor

        self.addTimerToSotControl = False

    def _newSoT (self, name):
        sot = SOT (name)
        sot.setSize(self.sotrobot.dynamic.getDimension())
        sot.damping.value = 0.001

        if self.addTimerToSotControl:
            from .tools import insertTimerOnOutput
            self.timers[name] = insertTimerOnOutput (sot.control, "vector")
            self.SoTtracer.add (self.timers[name].name + ".timer", str(len(self.timerIds)) + ".timer")
            self.timerIds.append(name)
        return sot

    def addAffordance (self, aff):
        assert isinstance(aff, Affordance)
        self.affordances [(aff.gripper, aff.handle)] = aff

    def generate (self):
        if self.addTimerToSotControl:
            # init tracer
            from dynamic_graph.tracer_real_time import TracerRealTime
            self.SoTtracer = TracerRealTime ("tracer_of_timers")
            self.timerIds = []
            self.SoTtracer.setBufferSize (10 * 1048576) # 10 Mo
            self.SoTtracer.open ("/tmp", "sot-control-trace", ".txt")
            self.sotrobot.device.after.addSignal("tracer_of_timers.triger")
            self.supervisor.SoTtracer = self.SoTtracer
            self.supervisor.SoTtimerIds = self.timerIds
        super(Factory, self).generate ()

        self.supervisor.sots = self.sots
        self.supervisor.grasps = { (gh, w): t for gh, ts in self.tasks._grasp.items() for w, t in ts.items() }
        self.supervisor.hpTasks = self.hpTasks
        self.supervisor.lpTasks = self.lpTasks
        self.supervisor.postActions = self.postActions
        self.supervisor.preActions  = self.preActions
        self.supervisor.SoTtimers = self.timers

        from dynamic_graph import plug
        self.supervisor.sots_indexes = dict()
        for tn,sot in self.sots.iteritems():
            # Pre action
            if self.preActions.has_key(tn):
                pa_sot = self.preActions[tn]
                self.supervisor.addSignalToSotSwitch (pa_sot.name, pa_sot.control)
            # Action
            if self.timers.has_key (sot.name):
                self.supervisor.addSignalToSotSwitch (sot.name, self.timers[sot.name].sout)
            else:
                self.supervisor.addSignalToSotSwitch (sot.name, sot.control)
            # Post action
            if self.postActions.has_key(tn):
                d = self.postActions[tn]
                for targetState, pa_sot in d.iteritems():
                    self.supervisor.addSignalToSotSwitch (pa_sot.name, pa_sot.control)

    def setupFrames (self, srdfGrippers, srdfHandles, sotrobot):
        self.sotrobot = sotrobot

        self.grippersIdx = { g: i for i,g in enumerate(self.grippers) }
        self.handlesIdx  = { h: i for i,h in enumerate(self.handles) }

        self.gripperFrames = { g: OpFrame(srdfGrippers[g], sotrobot.dynamic.model) for g in self.grippers }
        self.handleFrames  = { h: OpFrame(srdfHandles [h]                        ) for h in self.handles  }

    def makeState (self, grasps, priority):
        # Nothing to do here
        return Factory.State(self.tasks, grasps, self)

    def makeLoopTransition (self, state):
        n = self._loopTransitionName(state.grasps)
        sot = self._newSoT ('sot_'+n)

        self.hpTasks.pushTo(sot)
        state.manifold.pushTo(sot)
        self.lpTasks.pushTo(sot)

        self.sots[n] = sot

    def makeTransition (self, stateFrom, stateTo, ig):
        sf = stateFrom
        st = stateTo
        names = self._transitionNames(sf, st, ig)

        print "names: {}".format(names)

        iobj = self.objectFromHandle [st.grasps[ig]]
        obj = self.objects[iobj]
        noPlace = self._isObjectGrasped (sf.grasps, iobj)

        print "iobj, obj, noPlace: {}, {}, {}".format(iobj, obj, noPlace)

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
                self.hpTasks.pushTo(s)

                #if pregrasp and i == 1:
                    # Add pregrasp task
                    #self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'pregrasp').pushTo (s)
                if i < M: sf.manifold.pushTo(s)
                else:     st.manifold.pushTo(s)

                self.lpTasks.pushTo(s)
                self.sots[n] = s
                sots.append (n)

        key = sots[2*(M-1)]
        states = ( st.name, names[0] + "_intersec")

        sot = self._newSoT ("postAction_" + key)
        self.hpTasks.pushTo (sot)
        # self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'gripper_close').pushTo (sot)
        st.manifold.pushTo (sot)
        self.lpTasks.pushTo (sot)
        # print sot

        # TODO one post action is missing
        if not self.postActions.has_key(key):
            self.postActions[ key ] = dict()
        for n in states:
            self.postActions[ key ] [n] = sot

        key = sots[2*(M-1) + 1]
        states = ( st.name, names[0] + "_intersec")

        sot = self._newSoT ("preAction_" + key)
        self.hpTasks.pushTo (sot)
        # self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'gripper_close').pushTo (sot)
        sf.manifold.pushTo (sot)
        self.lpTasks.pushTo (sot)
        # print sot

        self.preActions[ key ] = sot
        self.preActions[ sots[2*(M-1)] ] = sot
