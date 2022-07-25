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

## Wrapper of a task in SoT and its interface with ROS
#
# This class represents a task as defined in the Stack of Tasks
# (dynamic_graph.sot.core.Task). It contains a dictionary the keys of which
# are the names of the SoT signals
#
# Each child class sets a default value for the gain.

class Task(object):
    sep = "___"

    def __init__ (self, tasks = [], constraints = [], topics = {}):
        ## Task to be added to a SoT solver
        self.tasks = list(tasks)
        ## Constraints
        # This is likely not used anymore.
        self.constraints = list(constraints)
        ## The ROS topics where to read references.
        # It is a dictionary. The key is the output signal name of a
        # RosQueuedSubscribe entity. The value is a dictionary with:
        # \li "type": "vector", "matrixHomo", "vector3" ...,
        # \li "topic": the ROS topic name,
        # \li "signalGetters": a list of functions returning the input signals to be plugged.
        # \li "handler": [optional] some specific topics have specific handlers.
        #     - "hppjoint": needs two more keys
        #       - "velocity": boolean
        #       - "hppjoint": HPP joint name
        #     - "tf_listener": needs three more keys
        #       - "velocity": boolean
        #       - "frame0": base frame name
        #       - "frame1": child frame name
        #     - "hppcom": needs two more keys
        #       - "velocity": boolean
        #       - "hppcom": HPP CoM name
        #
        #
        self.topics = dict(topics)
        ## Projector for the solver.
        # A projector is a NxK matrix, where N is the number of DoF of the
        # robot and K the dimension of the solver search space. This matrix
        # computes a robot velocity from a vector in the search space.
        # When not set, the search space is the robot velocity space.
        #
        # When stacking Task object, at most one of the Task can have a projector.
        self.projector = None

    def __add__ (self, other):
        res = Task(list(self.tasks), list(self.constraints), dict(self.topics))
        res.projector = other.projector
        res += other
        return res

    def __iadd__ (self, other):
        self.tasks += other.tasks
        self.constraints += other.constraints
        for k,v in other.topics.items():
            if k in self.topics.keys():
                a = self.topics[k]
                assert a["type"] == v["type"]
                if 'topic' in a.keys(): assert a["topic"] == v["topic"]
                else: assert a["handler"] == v["handler"]
                self.extendSignalGetters(k, v["signalGetters"])
                if 'defaultValue' in a.keys() and 'defaultValue' in v.keys():
                    from dynamic_graph.signal_base import SignalBase
                    if isinstance(a["defaultValue"], SignalBase):
                        assert a["defaultValue"].name == v["defaultValue"].name, \
                                "topics " + k + " cannot be merged because the default values are " \
                                + "different: \n" + str(a["defaultValue"]) \
                                + "\nand\n" + str(v["defaultValue"])
                    else:
                        assert a["defaultValue"] == v["defaultValue"], \
                                "topics " + k + " cannot be merged because the default values are " \
                                + "different: \n" + str(a["defaultValue"]) \
                                + "\nand\n" + str(v["defaultValue"])
                # print k, "has", len(a["signalGetters"]), "signals"
            else:
                self.topics[k] = v
        if self.projector is None:
            self.projector = other.projector
        elif other.projector is not None:
            raise ValueError('Cannot merge Task when both have a projector')
        return self

    def setControlSelection (self, selection):
        for t in self.tasks:
            t.controlSelec.value = selection

    def pushTo (self, action):
        """
        \param action an object of type agimus_sot.action.Action
        """
        for t in self.tasks:
            action.push(t)
        if self.projector is not None:
            action.setProjector(self.projector)

    def extendSignalGetters (self, topicName, signalGetters):
        """Add signal getters to a topic"""
        sgs = signalGetters if isinstance(signalGetters, (list, tuple, set, frozenset)) else [signalGetters,]
        topic =  self.topics[topicName]
        topic["signalGetters"] = topic["signalGetters"].union (sgs)

    def addHppJointTopic (self, topicName, jointName=None, velocity=False, signalGetters=frozenset()):
        """
        Add a topic that will received the pose (or velocity) of a joint from HPP
        - param jointName: When None, uses topicName as the joint name in HPP.
        """
        if topicName in self.topics:
            tp=self.topics[topicName]
            assert "velocity" in tp and tp['velocity'] ==  velocity
            assert "type"     in tp and tp["type"    ] ==  "matrixHomo"
            assert "handler"  in tp and tp["handler" ] ==  "hppjoint"
            assert "hppjoint" in tp and tp["hppjoint"] ==  jointName if jointName is not None else topicName
            self.extendSignalGetters(topicName, signalGetters)
            return
        self.topics[topicName] = {
                "velocity": velocity,
                "type": "vector" if velocity else "matrixHomo",
                "handler": "hppjoint",
                "hppjoint": jointName if jointName is not None else topicName,
                "signalGetters": frozenset(signalGetters),
                }

    ## \param signals should be a set of:
    ##        - either a tuple (signal, availability signal)
    ##        - or a signal
    def addTfListenerTopic (self, topicName, frame0, frame1,
            defaultValue=None, signalGetters=frozenset(),
            maxDelay=1.5):
        if topicName in self.topics:
            tp=self.topics[topicName]
            assert "velocity" in tp and tp['velocity'] ==  False
            assert "type"     in tp and tp["type"    ] ==  "matrixHomo"
            assert "handler"  in tp and tp["handler" ] ==  "tf_listener"
            assert "frame0"   in tp and tp["frame0"  ] ==  frame0
            assert "frame1"   in tp and tp["frame1"  ] ==  frame1
            assert "maxDelay" in tp and tp["maxDelay"] ==  maxDelay
            self.extendSignalGetters(topicName, signalGetters)
            return
        self.topics[topicName] = {
                "velocity": False,
                "type": "matrixHomo",
                "handler": "tf_listener",
                "frame0": frame0,
                "frame1": frame1,
                "signalGetters": frozenset(signalGetters),
                "maxDelay": maxDelay
                }
        if defaultValue is not None:
            self.topics[topicName]["defaultValue"] = defaultValue

    ## Create an If entity whose condition is _the transform was found in TF_.
    ## If the condition is not met, the \c value is used instead.
    ## \param value either a SignalBase or a transform.
    ## \param outputs a signal or a list of signal that will receive the output
    ##        of the If entity.
    ## \return a 2-uplet:
    ##         - the If entity, whose signal sin1 (resp. boolSelection) should be
    ##           plugged to tf output signal (resp tf availability signal)
    ##         - a tuple that should be passed to addTfListenerTopic
    def makeTfListenerDefaultValue (self, name, value, outputs = None):
        from dynamic_graph.sot.core.switch import SwitchMatrixHomogeneous as Switch
        from agimus_sot.tools import plugMatrixHomo, assertEntityDoesNotExist
        from dynamic_graph.signal_base import SignalBase
        from dynamic_graph import plug

        assertEntityDoesNotExist(name)
        switch = Switch(name)
        switch.setSignalNumber(2)
        plugMatrixHomo(value, switch.sin(0))
        if outputs is not None:
            if isinstance(outputs, SignalBase):
                plug(switch.sout, outputs)
            elif isinstance(outputs, (list, tuple)):
                for output in outputs:
                    plug(switch.sout, output)
        return switch, (switch.sin(1), switch.boolSelection)

    def addTrace (self, tracer):
        from agimus_sot.tools import filename_escape
        for task in self.tasks:
            tracer.add (task.name + ".error", filename_escape(task.name) + ".error")

    def _name (self, *args):
        return self.sep.join ((self.name_prefix,) + args)
