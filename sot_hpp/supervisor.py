from tools import Grasp, OpFrame, Manifold, Posture
from dynamic_graph.sot.core import SOT
from dynamic_graph import plug

def idx(l): return range(len(l))
def idx_zip (l): return zip (idx(l), l)

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
    def __init__ (self, hppclient, sotrobot, grippers, objects, handlesPerObjects):
        handles = [ h for i in idx(objects) for h in handlesPerObjects[i] ]
        self.hpp = hppclient
        self.sotrobot = sotrobot
        self.grippers = [ OpFrame(hppclient) for _ in idx(grippers)]
        self.handles  = [ OpFrame(hppclient) for _ in idx(handles )]
        self.grippersIdx = { grippers[i] : i for i in idx(grippers) }
        self.handlesIdx  = {  handles[i] : i for i in idx(handles) }

        for ig, g in idx_zip(grippers): self.grippers[ig].setHppGripper (g)
        for ih, h in idx_zip(handles ): self.handles [ih].setHppHandle  (h)

        for g in self.grippers: g.setSotFrameFromHpp (self.sotrobot.dynamic.model)
        for h in self.handles : h.setSotFrameFromHpp (self.sotrobot.dynamic.model)

        self.postureTask = Posture ("posture", sotrobot)
        self.hpTasks = Manifold()
        self.lpTasks = Posture ("posture", sotrobot)

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
        self.transitions = transitions

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

    def plugTopics (self, rosexport):
        c = self.hpTasks + self.lpTasks
        for g in self.grasps.values():
            c += g

        for n, t in c.topics.items():
            if t.has_key('handler'):
                topic = _handlers[t['handler']] (n, t)
            else:
                topic = t["topic"]
            rosexport.add (t["type"], n, topic)
            for s in t['signalGetters']:
                plug (rosexport.signal(n), s())
            print topic, "plugged to", n

    def isSotConsistentWithCurrent(self, id, thr = 1e-3):
        csot = self.sots[self.currentSot]
        nsot = self.sots[id]
        t = csot.control.time
        nsot.recompute (t)
        nsot.control.recompute(t)
        from numpy import array, linalg
        error = array(nsot.control.value) - array(csot.control.value)
        n = linalg.norm(error)
        if n > thr:
            print "Control not consistent:", error
            return False
        return True

    def plugSot(self, id):
        sot = self.sots[id]
        plug(sot.control, self.sotrobot.device.control)
        self.currentSot = id

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
    return "hpp/target/" + topic + '/' + t['hppjoint']

_handlers = {
        "hppjoint": _handleHppJoint,
        }
