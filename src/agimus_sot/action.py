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

##
#  Action associated to a transition pre-action, action, or post-ation
#
#  An action is composed of
#  \li a list of pre-actions, i.e. fonctions that are called sequentially;
#      if one of them return false, the action returns failure,
#  \li a controller of type SOT (stack of task) that is activated upon
#      completion of the preactions,

class Action(object):
    maxControlSqrNorm = 10.
    def __init__ (self, name, dimension, damping = None, timer = False):
        from dynamic_graph.entity import VerbosityLevel
        from dynamic_graph.sot.core.sot import SOT
        # Initialize list of pre-actions and post-actions
        self.preActions = list()
        sot = SOT(name)
        sot.setSize(dimension)
        sot.setMaxControlIncrementSquaredNorm(self.maxControlSqrNorm)
        sot.setLoggerVerbosityLevel(VerbosityLevel.VERBOSITY_ALL)
        if damping is not None: sot.damping.value = damping

        self.sot = sot
        self.tasks = []
        if timer:
            from .tools import insertTimerOnOutput
            self.timer = insertTimerOnOutput (sot.control, "vector")
        else:
            self.timer = None

    def push (self, task):
        """
        task: an object of type agimus_sot.task.Task
        """
        self.sot.push(task.name)
        self.tasks.append(task)

    def setProjector (self, projector):
        """
        projector: a signal of type matrix type

        A projector is a NxK matrix, where N is the number of DoF of the
        robot and K the dimension of the action search space. This matrix
        computes a robot velocity from a vector in the search space.
        When not set, the search space is the robot velocity space.
        """
        from dynamic_graph import plug
        plug(projector, self.sot.proj0)

    def runPreactions(self):
        for action in self.preActions:
            # trigger error if action returns false
            res, msg = action()
            if not res:
                self.errorSignal = False
                return False, msg
        return True, ""

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
    def controlname (self):
        if self.timer is None: return self.sot.name + ".control"
        else                 : return self.timer.name + ".sout"

    @property
    def name (self): return self.sot.name
