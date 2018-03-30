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
            otherIg = self.graphfactory.grippers.index(otherGrasp.gripper.name)
            otherIh = self.graphfactory.handles.index(otherGrasp.handle.name)
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

        self.supervisor = supervisor

    def addAffordance (self, aff):
        assert isinstance(aff, Affordance)
        self.affordances [(aff.gripper, aff.handle)] = aff

    def finalize (self, hppclient):
        graph, elmts = hppclient.manipulation.graph.getGraph()
        ids = { n.name: n.id for n in elmts.edges }
        nids = { n.name: n.id for n in elmts.nodes }

        self.transitionIds = { n.name: n.id for n in elmts.edges }
        # self.supervisor.sots = { ids[n]: sot for n, sot in self.sots.items() if ids.has_key(n) }
        # self.supervisor.grasps = { (gh, w): t for gh, ts in self.tasks._grasp.items() for w, t in ts.items() }
        # self.supervisor.hpTasks = self.hpTasks
        # self.supervisor.lpTasks = self.lpTasks
        # self.supervisor.postActions = {
                # ids[trans] : {
                    # nids[state]: sot for state, sot in values.items() if nids.has_key(state)
                    # } for trans, values in self.postActions.items() if ids.has_key(trans)
                # }
        # self.supervisor.preActions = { ids[trans] : sot for trans, sot in self.preActions.items() if ids.has_key(trans) }
        self.supervisor.sots = self.sots
        self.supervisor.grasps = { (gh, w): t for gh, ts in self.tasks._grasp.items() for w, t in ts.items() }
        self.supervisor.hpTasks = self.hpTasks
        self.supervisor.lpTasks = self.lpTasks
        self.supervisor.postActions = self.postActions
        self.supervisor.preActions  = self.preActions

    def setupFrames (self, hppclient, sotrobot):
        self.sotrobot = sotrobot

        self.grippersIdx = { g: i for i,g in enumerate(self.grippers) }
        self.handlesIdx  = { h: i for i,h in enumerate(self.handles) }

        self.gripperFrames = { g: OpFrame(hppclient) for g in self.grippers }
        self.handleFrames  = { h: OpFrame(hppclient) for h in self.handles  }

        for g in self.grippers: self.gripperFrames[g].setHppGripper (g)
        for h in self.handles : self.handleFrames [h].setHppHandle  (h)

        for g in self.gripperFrames.values(): g.setSotFrameFromHpp (sotrobot.dynamic.model)
        for h in self.handleFrames .values(): h.setSotFrameFromHpp (sotrobot.dynamic.model)

    def makeState (self, grasps, priority):
        # Nothing to do here
        return Factory.State(self.tasks, grasps, self)

    def makeLoopTransition (self, state):
        n = self._loopTransitionName(state.grasps)
        sot = SOT ('sot_' + n)
        sot.setSize(self.sotrobot.dynamic.getDimension())

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
                s = SOT ('sot_' + n)
                s.setSize(self.sotrobot.dynamic.getDimension())
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

        sot = SOT ("postAction_" + key)
        sot.setSize(self.sotrobot.dynamic.getDimension())
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

        sot = SOT ("preAction_" + key)
        sot.setSize(self.sotrobot.dynamic.getDimension())
        self.hpTasks.pushTo (sot)
        # self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'gripper_close').pushTo (sot)
        sf.manifold.pushTo (sot)
        self.lpTasks.pushTo (sot)
        # print sot

        self.preActions[ key ] = sot
        self.preActions[ sots[2*(M-1)] ] = sot
