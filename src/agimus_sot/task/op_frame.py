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

from .task import Task
from agimus_sot.tools import transQuatToSE3

## Represents a gripper or a handle
class OpFrame(object):
    def __init__ (self, srdf, model = None, enabled = None):
        self.robotName = srdf["robot"]
        self.name = srdf["name"]
        self.key = self.robotName + "/" + self.name
        self.link = srdf["link"]
        self.lMf = transQuatToSE3 (srdf["position"])
        self.enabled = enabled
        if srdf.has_key ("joints"):
            assert model is not None
            ## Only for grippers
            self.joints = srdf["joints"]
            self._setupParentJoint (self.link, self.lMf, model)
            if self.enabled is None:
                self.enabled = True
            if srdf.has_key("torque_constant"):
                self.torque_constant = srdf["torque_constant"]
        else:
            ## Only for handles
            # kept for backward compat
            self.pose = self.lMf
            if self.enabled is None:
                self.enabled = False
        # TODO See note in README.md
        self.hasVisualTag = False

    def _setupParentJoint (self, link, pose, model):
        frameid = model.getFrameId (link)
        if frameid < 0 or frameid >= len(model.frames):
            raise ValueError("Link " + self.link + " not found")
        frame = model.frames[frameid]

        # kept for backward compat
        self.pose = frame.placement * pose
        self.jMf = self.pose
        self.joint = model.names[frame.parent]

    @property
    def fullLink  (self): return self.robotName + "/" + self.link
    @property
    def fullJoint (self): return self.robotName + "/" + self.joint
    @property
    def fullName  (self): return self.robotName + "/" + self.name
