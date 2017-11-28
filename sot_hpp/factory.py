from hpp.corbaserver.manipulation.constraint_graph_factory import ConstraintFactoryAbstract, GraphFactoryAbstract
from tools import Manifold, Grasp, idx, idx_zip, OpFrame, EEPosture
from dynamic_graph.sot.core import SOT

class TaskFactory(ConstraintFactoryAbstract):
    gfields = ('grasp', 'pregrasp', 'gripper_open', 'gripper_close')
    pfields = ()

    def __init__ (self, graphfactory):
        super(TaskFactory, self).__init__ (graphfactory)

    def buildGrasp (self, g, h):
        gf = self.graphfactory
        if h is None:
            return { 'gripper_open': EEPosture (gf.sotrobot, gf.gripperFrames [g], [0]) }

        gripper_close  = EEPosture (gf.sotrobot, gf.gripperFrames [g], [-1])
        pregrasp = Grasp (gf.gripperFrames [g],
                          gf.handleFrames [h])
        pregrasp.makeTasks (gf.sotrobot)
        grasp = Grasp (gf.gripperFrames [g],
                       gf.handleFrames [h])
        grasp.makeTasks (gf.sotrobot)
        return { 'grasp': grasp,
                 'pregrasp': pregrasp,
                 'gripper_close': gripper_close }

    def buildPlacement (self, o):
        # Nothing to do
        return dict()

class Factory(GraphFactoryAbstract):
    class State:
        def __init__ (self, tasks, grasps, factory):
            self.name = factory._stateName (grasps)
            self.grasps = grasps
            self.manifold = Manifold()

            for ig, ih in idx_zip (grasps):
                if ih is not None:
                    # Add task gripper_close
                    self.manifold += tasks.g (factory.grippers[ig], factory.handles[ih], 'gripper_close')
                    # TODO If an object is grasped by two grippers, then we should add a task
                    # of relative position of the two grippers.
                else:
                    # Add task gripper_open
                    self.manifold += tasks.g (factory.grippers[ig], None, 'gripper_open')

    def __init__ (self, supervisor):
        super(Factory, self).__init__ ()
        self.tasks = TaskFactory (self)
        self.hpTasks = supervisor.hpTasks
        self.lpTasks = supervisor.lpTasks
        self.sots = dict()
        self.postActions = dict()
        self.preActions = dict()

        self.supervisor = supervisor

    def finalize (self, hppclient):
        graph, elmts = hppclient.manipulation.graph.getGraph()
        ids = { n.name: n.id for n in elmts.edges }
        nids = { n.name: n.id for n in elmts.nodes }

        self.supervisor.sots = { ids[n]: sot for n, sot in self.sots.items() }
        self.supervisor.grasps = { (gh, w): t for gh, ts in self.tasks._grasp.items() for w, t in ts.items() }
        self.supervisor.hpTasks = self.hpTasks
        self.supervisor.lpTasks = self.lpTasks
        self.supervisor.postActions = {
                ids[trans] : {
                    nids[state]: sot for state, sot in values.items() if nids.has_key(state)
                    } for trans, values in self.postActions.items()
                }
        self.supervisor.preActions = { ids[trans] : sot for trans, sot in self.preActions.items() }

    def setupFrames (self, hppclient, sotrobot):
        self.sotrobot = sotrobot

        self.grippersIdx = { self.grippers[i] : i for i in idx(self.grippers) }
        self.handlesIdx  = {  self.handles[i] : i for i in idx(self.handles) }

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

        # TODO Add the notion of pre/post actions
        # to open and close the gripper

        iobj = self.objectFromHandle [st.grasps[ig]]
        obj = self.objects[iobj]
        noPlace = self._isObjectGrasped (sf.grasps, iobj)

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

                if pregrasp and i == 1:
                    # Add pregrasp task
                    self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'pregrasp').pushTo (s)
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
