from tools import Grasp, OpFrame, Manifold, Posture, idx, idx_zip
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

    def initForGrasps (self, hppclient, grippers, objects, handlesPerObjects):
        handles = [ h for i in idx(objects) for h in handlesPerObjects[i] ]
        self.grippers = [ OpFrame(hppclient) for _ in idx(grippers)]
        self.handles  = [ OpFrame(hppclient) for _ in idx(handles )]
        self.grippersIdx = { grippers[i] : i for i in idx(grippers) }
        self.handlesIdx  = {  handles[i] : i for i in idx(handles) }

        for ig, g in idx_zip(grippers): self.grippers[ig].setHppGripper (g)
        for ih, h in idx_zip(handles ): self.handles [ih].setHppHandle  (h)

        for g in self.grippers: g.setSotFrameFromHpp (self.sotrobot.dynamic.model)
        for h in self.handles : h.setSotFrameFromHpp (self.sotrobot.dynamic.model)

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

    def makeGrasps (self, transitions):
        """
        @param transition: a list of dictionaries containing the following keys:
                        - "id", "name": the id and name of the transition in hpp.
                        - "manifold": a tuple of (gripper_index, handle_index) defining the submanifold into which the transition takes place.
                        - "grasp" (optional) : (gripper_index, handle_index) corresponding to the added/removed grasp.
                        - "forward" (required if "grasp") : True if added, False if removed.
                        - "step" (required if "grasp") : integer [ 0, 5 ].
        """
        self.grasps = dict()
        self.sots = dict()
        self.postActions = dict()
        self.preActions = dict()
        self.transitions = transitions

        print "Transitions:\n"
        print transitions
        print "\n"

        for t in transitions:
            # Create SOT solver
            # sot = SOT ('sot_' + str(t['id']) + '-' + t['name'])
            sot = SOT ('sot_' + str(t['id']))
            sot.setSize(self.sotrobot.dynamic.getDimension())

            # Push high priority tasks (Equilibrium for instance)
            self.hpTasks.pushTo(sot)

            # Create (or get) the tasks
            M = self._manifold(t["manifold"])
            if t.has_key("grasp"):
                grasp = self._manifold(t["manifold"] + (t["grasp"],))
                forward = bool(t["forward"])
                step = int(t["step"])
                assert step >= 0 and step <= 5, "'step' must be an integer within [0, 5]"
                # TODO Events should be set up here for each step.
                # For instance, adding some events based on force feedback to say
                # when the object is grasped.
                if forward:
                    if step == 1:
                        M.pushTo(sot)
                    elif step == 0 or step == 2:
                        grasp.pushTo(sot)
                    else:
                        grasp.pushTo(sot)
                else:
                    if step == 1:
                        M.pushTo(sot)
                    elif step == 0 or step == 2:
                        grasp.pushTo(sot)
                    else:
                        grasp.pushTo(sot)
            else:
                M.pushTo(sot)

            # Put low priority tasks
            self.lpTasks.pushTo(sot)

            self.sots[t['id']] = sot

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
        self.keep_posture._signalPositionRef().value = tuple([-0.74, 0.0, 1.0, 0.0, 0.0, 0.0] + list(self.sotrobot.dynamic.position.value)[6:])
        
        self.keep_posture.pushTo(sot)
        self.sots[-1] = sot

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
            print topic, "plugged to", n, ', ', len(t['signalGetters']), 'times'

    def setupReferences (self, gui, rosexport):
        ret = dict()
        gui.createGroup("references")
        topics = self.topics()
        for n, t in topics.items():
            types = []
            if t.has_key('handler'):
                if t['handler'] == 'hppjoint':
                    if t['velocity']:
                        types = ['linvel', 'angvel']
                    else:
                        types = ['pose',]
                elif t['handler'] == 'hppcom':
                    if t['velocity']:
                        types = ['linvel',]
                    else:
                        types = ['point',]
            name = "ref_" + n
            ret[n] = list()
            for i,type in idx_zip(types):
                nameref = name + str(i)
                if type == 'pose':
                    gui.addXYZaxis(nameref, (1,0,0,1), 0.005, 0.05)
                    gui.setVisibility(nameref, 'ALWAYS_ON_TOP')
                    gui.addToGroup(nameref, "references")
                    ret[n].append(type)
                # elif type == '':
        return ret

    def setReferenceValue (self, gui, rosexport, refs):
        from dynamic_graph.sot.tools.quaternion import Quaternion
        from dynamic_graph.sot.tools.se3 import SE3
        idx = 0
        for n, types in refs.items():
            for i,type in idx_zip(types):
                nameref = "ref_" + n + str(i)
                sig = rosexport.signal(n)
                # if sig.time != self.sotrobot.device.control.time or len(sig.value) == 0:
                if len(sig.value) == 0:
                    if gui.getIntProperty(nameref, "visibility") != 2:
                        print "Hiding", nameref
                        gui.setVisibility(nameref, 'OFF')
                    continue
                else:
                    if gui.getIntProperty(nameref, "visibility") == 2:
                        print "Showing", nameref
                        # gui.setVisibility(nameref, 'ON')
                        gui.setVisibility(nameref, 'ALWAYS_ON_TOP')
                if type == 'pose':
                    H = SE3(sig.value)
                    q = Quaternion(sig.value)
                    gui.applyConfiguration(nameref, list(H.translation.value) + list(q.array[1:]) + list(q.array[:1]) )
        gui.refresh()

    def isSotConsistentWithCurrent(self, id, thr = 1e-3):
        if self.currentSot is None or id == self.currentSot:
            return True
        csot = self.sots[self.currentSot]
        nsot = self.sots[id]
        t = self.sotrobot.device.control.time
        csot.control.recompute(t)
        nsot.control.recompute(t)
        from numpy import array, linalg
        error = array(nsot.control.value) - array(csot.control.value)
        n = linalg.norm(error)
        if n > thr:
            print "Control not consistent:", linalg.norm(error)
            print error
            return False
        return True

    def clearQueues(self):
        exec ("tmp = " + self.rosexport.list())
        for s in tmp:
            self.rosexport.clearQueue(s)

    def readQueue(self, read):
        if read < 0:
            print "ReadQueue argument should be >= 0"
            return
        t = self.sotrobot.device.control.time
        self.rosexport.readQueue (t + read)

    def stopReadingQueue(self):
        self.rosexport.readQueue (-1)

    def plugSot(self, id, check = False):
        if check and not self.isSotConsistentWithCurrent (id):
            # raise Exception ("Sot %d not consistent with sot %d" % (self.currentSot, id))
            print "Sot %d not consistent with sot %d" % (self.currentSot, id)
        if id == -1:
            # TODO : Explanation and linked TODO in the function makeInitialSot
            if self.sotrobot.dynamic.position.value[0] > -0.5:
                self.keep_posture._signalPositionRef().value = tuple([-0.74, 0.0, 1.0, 0.0, 0.0, 0.0] + list(self.sotrobot.dynamic.position.value)[6:])
            else:
                self.keep_posture._signalPositionRef().value = self.sotrobot.dynamic.position.value
        sot = self.sots[id]
        # Start reading queues
        self.readQueue(10)
        plug(sot.control, self.sotrobot.device.control)
        print "Current sot:", id
        print sot.display()
        self.currentSot = id

    def runPreAction(self, idTransition):
        if self.preActions.has_key(idTransition):
            sot = self.preActions[idTransition]
            print "Running pre action", idTransition
            print sot.display()
            t = self.sotrobot.device.control.time
            sot.control.recompute(t-1)
            plug(sot.control, self.sotrobot.device.control)
            return
        print "No pre action", idTransition

    def runPostAction(self, idStateTarget):
        if self.postActions.has_key(self.currentSot):
            d = self.postActions[self.currentSot]
            if d.has_key(idStateTarget):
                sot = d[idStateTarget]
                print "Running post action", self.currentSot, idStateTarget
                print sot.display()
                t = self.sotrobot.device.control.time
                sot.control.recompute(t-1)
                plug(sot.control, self.sotrobot.device.control)
                return
        print "No post action", self.currentSot, idStateTarget

    def getJointList (self, prefix = ""):
        return [ prefix + n for n in self.sotrobot.dynamic.model.names[1:] ]

    def _manifold (self, idxs):
        if self.grasps.has_key(idxs):
            return self.grasps[idxs]
        if len(idxs) == 1:
            m = Grasp(self.grippers[idxs[0][0]], self.handles[idxs[0][1]])
            m.makeTasks(self.sotrobot)
        elif len(idxs) == 0:
            m = Manifold()
        else:
            subm = self._manifold(idxs[:-1])
            # TODO Find relative
            m = Grasp(self.grippers[idxs[-1][0]], self.handles[idxs[-1][1]])
            m.makeTasks(self.sotrobot)
            m += subm
        self.grasps[idxs] = m
        return m

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
