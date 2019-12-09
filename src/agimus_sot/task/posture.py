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
from dynamic_graph.sot.core import FeaturePosture, GainAdaptive, Task as SotTask
from dynamic_graph.sot.core.meta_tasks import setGain

from .task import Task

## Postural task
class Posture(Task):
    def __init__ (self, name, sotrobot, withDerivative = False):
        super(Posture, self).__init__()

        n = Posture.sep + name
        self.tp = SotTask ('task' + n)
        self.tp.dyn = sotrobot.dynamic
        self.tp.feature = FeaturePosture('feature_'+n)

        q = list(sotrobot.dynamic.position.value)
        self.tp.feature.state.value = q
        self.tp.feature.posture.value = q

        robotDim = sotrobot.dynamic.getDimension()
        for i in range(6, robotDim):
            self.tp.feature.selectDof (i, True)
        self.tp.gain = GainAdaptive("gain_"+n)
        self.tp.add(self.tp.feature.name)

        # Connects the dynamics to the current feature of the posture task
        plug(sotrobot.dynamic.position, self.tp.feature.state)

        self.tp.setWithDerivative (withDerivative)

        # Set the gain of the posture task
        setGain(self.tp.gain,(4.9,0.9,0.01,0.9))
        plug(self.tp.gain.gain, self.tp.controlGain)
        plug(self.tp.error, self.tp.gain.error)
        self.tasks = [ self.tp ]
        self.topics = {
                    name: {
                        "type": "vector",
                        "topic": "/hpp/target/position",
                        "signalGetters": frozenset([ self._signalPositionRef ]) },
                }
        if withDerivative:
            self.topics["vel_" + name] = {
                    "type": "vector",
                    "topic": "/hpp/target/velocity",
                    "signalGetters": frozenset([ self._signalVelocityRef ])
                    }

    def _signalPositionRef (self): return self.tp.feature.posture
    def _signalVelocityRef (self): return self.tp.feature.postureDot

