# Copyright 2021 CNRS - Airbus SAS
# Author: Florent Lamiraux
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
from agimus_sot.task import Task, Grasp, PreGraspPostAction, OpFrame, EndEffector
from agimus_sot.solver import Solver
from .pre_grasp import PreGrasp

## Create \ref task.Task s
#
# \sa manipulation.constraint_graph_factory.ConstraintFactoryAbstract
class TaskFactory(ConstraintFactoryAbstract):
    gfields = ('grasp', 'pregrasp', 'gripper_open', 'gripper_close')
    pfields = ('preplace',)

    def __init__(self, graphfactory):
        super(TaskFactory, self).__init__ (graphfactory)
        self._grippers = dict()
        self._grasps = dict()
        self._placements = dict()

    def _buildGripper (self, type, gripper, handle):
        if type == "open" or type  == "close":
            return Task()
        assert(false)

    def buildGrasp(self, g, h, otherGrasp=None):
        gf = self.graphfactory
        if h is None:
            gripper_open = self._buildGripper ("open", g, h)
            return { 'gripper_open': gripper_open }
        gripper = gf.gripperFrames[g]
        handle  = gf.handleFrames [h]

        gripper_close = self._buildGripper ("close", g, h)
        pregrasp = PreGrasp (gripper, handle, otherGrasp)
        pregrasp.makeTasks (gf.sotrobot)
        if gf.parameters["addTracerToVisualServoing"] and \
                (useMeasurementOfObjectPose \
                or useMeasurementOfGripperPose \
                or useMeasurementOfOtherGripperPose):
            pregrasp.addVisualServoingTrace (gf.ViStracer)

        pregrasp_pa = PreGraspPostAction (gripper, otherGrasp[0] if otherGrasp is not None else None)
        pregrasp_pa.makeTasks (gf.sotrobot)

        if not gripper.enabled:
            # TODO If otherGrasp is not None,
            # we should include the grasp function of otherGrasp, not pregrasp function...
            grasp = Task()
        else:
            grasp = Grasp (gripper, handle, otherGrasp)
            grasp.makeTasks (gf.sotrobot)
        return { 'grasp': grasp,
                 'pregrasp': pregrasp,
                 'pregrasp_postaction': pregrasp_pa,
                 'gripper_close': gripper_close }

    
    def buildPlacement(self, o, grasp):
        return {'preplace': Task(), 'preplace_postaction': Task()}
    
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

    def getPlacement(self, object, grasp):
        if isinstance(object, str): io = self.graphfactory.objects.index(object)
        else: io = object
        ig = self.graphfactory.grippers.index(grasp[0].key)
        ih = self.graphfactory.handles .index(grasp[1].key)
        k = (io, ig, ih,)
        if not self._placements.has_key(k):
            self._placements[k] = self.buildPlacement(self.graphfactory.objects[io], grasp)
            assert isinstance (self._placements[k], dict)
        return self._placements[k]

    def p (self, object, grasp, what):
        return self.getPlacement(object, grasp)[what]

    def event (self, gripper, handle, what, default):
        if handle is None:
            ee = self._buildGripper ("open", gripper, handle)
        else:
            ee = self._buildGripper ("close", gripper, handle)
        if hasattr(ee, "events"):
            return ee.events.get(what, default)
        return default
