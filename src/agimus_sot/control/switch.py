# Copyright 2018 CNRS - Airbus SAS
# Author: Joseph Mirabel
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

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.core.switch import SwitchVector
from dynamic_graph.sot.core.latch import Latch
from dynamic_graph.sot.core.event import Event
from dynamic_graph.sot.core.operator import CompareVector

class ControllerSwitch:
    def __init__ (self,name,controllers,threshold_up,threshold_down):
        """
        Use controller 0 until the condition signal value becomes greater than threshold.
        Then use controller 1. Manually switch between controller using the latch.
        OFF means controller 0 and ON means controller 1.

        Currently support only two controllers.
        - controllers: output signal of a controller
        """
        self.reverse = (threshold_up < threshold_down)

        # any(threshold_up < measured_torque) => torque control
        self._condition_up = CompareVector (name + "_condition_up")
        self._condition_up.setTrueIfAny(True)
        self._condition_up.sin1.value = np.array(threshold_up)
        self._condition_up.sin2.value = np.array(threshold_up)

        # all(measured_torque < threshold_down) => position control
        self._condition_down = CompareVector (name + "_condition_down")
        self._condition_down.setTrueIfAny(False)
        self._condition_down.sin1.value = np.array(threshold_down)
        self._condition_down.sin2.value = np.array(threshold_down)

        self._event_up   = Event (name + "_event_up")
        self._event_down = Event (name + "_event_down")
        self._event_up   .setOnlyUp(True);
        self._event_down .setOnlyUp(True);
        self._latch = Latch(name + "_latch")
        self._latch.turnOff()

        self._switch = SwitchVector (name + "_switch")
        self._switch.setSignalNumber(len(controllers))

        plug(self._condition_up  .sout, self._event_up  .condition)
        plug(self._condition_down.sout, self._event_down.condition)
        plug(self._latch.out , self._switch.boolSelection)

        # This is necessary to initialize the event (the first recompute triggers the event...)
        self._event_up.check.recompute(0)
        self._event_down.check.recompute(0)
        self._event_up  .addSignal (name + "_latch.turnOnSout")
        self._event_down.addSignal (name + "_latch.turnOffSout")

        for n, sig in enumerate(controllers):
            plug(sig, self.signalIn(n))

    def setMeasurement (self,sig):
        if self.reverse:
            plug(sig, self._condition_up  .sin1)
            plug(sig, self._condition_down.sin2)
        else:
            plug(sig, self._condition_up  .sin2)
            plug(sig, self._condition_down.sin1)
    @property
    def thresholdUp   (self):
        if self.reverse:
            return self._condition_up  .sin2
        else:
            return self._condition_up  .sin1
    @property
    def thresholdDown (self):
        if self.reverse:
            return self._condition_down.sin1
        else:
            return self._condition_down.sin2
    @property
    def signalOut (self): return self._switch.sout

    def signalIn (self, n): return self._switch.signal("sin" + str(n))

    @property
    def conditionUp   (self): return self._condition_up
    @property
    def conditionDown (self): return self._condition_down
    @property
    def eventUp   (self): return self._event_up
    @property
    def eventDown (self): return self._event_down
    @property
    def latch (self): return self._latch
    @property
    def switch (self): return self._switch
