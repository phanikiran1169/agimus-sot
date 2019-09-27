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

class Solver(object):
    def __init__ (self, name, dimension, damping = None, timer = None):
        from dynamic_graph.sot.core import SOT
        sot = SOT(name)
        sot.setSize(dimension)
        if damping is not None: sot.damping.value = damping

        self.sot = sot
        self.tasks = []
        if timer:
            from .tools import insertTimerOnOutput
            self.timer = insertTimerOnOutput (sot.control, "vector")
        else:
            self.timer = None

    ## \name Events
    # \{

    ## A boolean signal which is True when the task is accomplished.
    #
    # If it is of type bool, the corresponding signal is assumed to have this value.
    @property
    def doneSignal (self): return self._doneSignal
    @doneSignal.setter
    def doneSignal (self, sig): self._doneSignal = sig

    ## A boolean signal which is True when an error occured.
    #
    # If it is of type bool, the corresponding signal is assumed to have this value.
    @property
    def errorSignal (self): return self._errorSignal
    @errorSignal.setter
    def errorSignal (self, sig): self._errorSignal = sig

    ## \}

    @property
    def control (self):
        if self.timer is None: return self.sot.control
        else                 : return self.timer.sout

    @property
    def name (self): return self.sot.name
