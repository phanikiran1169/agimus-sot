# Copyright 2018, 2019 CNRS - Airbus SAS
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

from dynamic_graph import plug
from dynamic_graph.sot.core import FeaturePose, GainAdaptive, Task as SotTask
from dynamic_graph.sot.core.meta_tasks import setGain

from .task import Task
from agimus_sot.tools import se3ToTuple, _createOpPoint

## A grasp task
# It creates a grasp constraint only in the case where
# an object is grasped by two grippers.
class Grasp (Task):
    def __init__ (self, gripper, handle, otherGraspOnObject = None):
        super(Grasp, self).__init__()
        self.gripper = gripper
        self.handle = handle

        # self.relative = otherGraspOnObject is not None \
                # and otherGraspOnObject.handle.link == self.handle.link
        # TODO: We should make sure that the relative position is constant
        # but we have no way to do it (HPP does). We assume that the object is
        # not articulated.
        self.relative = (otherGraspOnObject is not None)
        if self.relative:
            self.otherGripper = otherGraspOnObject[0]
            self.otherHandle = otherGraspOnObject[1]

    ## \todo implement tracking of velocity
    # \c self.task.task.setWithDerivative(True)
    # which needs the following signals:
    # - velocity of object attached to gripper.sotjoint, into self.task.feature.dotposition
    # - velocity of object attached to otherGripper.sotjoint, into self.task.feature.dotpositionRef
    def makeTasks(self, sotrobot, withDerivative = False):
        if self.relative:
            basename = Grasp.sep.join([self.gripper.name, self.otherGripper.name])

            # Create the FeaturePose
            self.feature = FeaturePose (basename+"_feature")
            #self.feature.jaMfa.value = \
            #        se3ToTuple(self.otherGripper.pose * self.otherHandle.pose.inverse())
            #self.feature.jbMfb.value = \
            #        se3ToTuple(self.gripper     .pose * self.handle     .pose.inverse())

            # Create the operational points
            _createOpPoint (sotrobot, self.     gripper.link)
            _createOpPoint (sotrobot, self.otherGripper.link)

            dyn = sotrobot.dynamic
            plug(dyn.signal(    self.otherGripper.link), self.feature. oMja)
            plug(dyn.signal(    self.     gripper.link), self.feature. oMjb)
            plug(dyn.signal('J'+self.otherGripper.link), self.feature.jaJja)
            plug(dyn.signal('J'+self.     gripper.link), self.feature.jbJjb)

            self.feature.faMfbDes.value = \
                    se3ToTuple(self.otherHandle .lMf
                            *  self.otherGripper.lMf.inverse()
                            *  self.     gripper.lMf
                            *  self.     handle .lMf.inverse())

            # Wrap it into a Task
            self.task = SotTask (basename+"_task")
            self.gain = GainAdaptive (basename+"_gain")

            self.task.add (self.feature.name)
            plug (self.task.error, self.gain.error)
            plug (self.gain.gain , self.task.controlGain)

            setGain(self.gain,(4.9,0.9,0.01,0.9))
            if withDerivative:
                print("Grasp constraint with derivative is not implemented yet.")
            self.task.setWithDerivative (False)

            self.tasks = [ self.task ]
