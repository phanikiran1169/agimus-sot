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

import numpy as np
from dynamic_graph import plug
from agimus_sot.sot import SafeGainAdaptive
from agimus_sot.task import Task, SotTask, FeaturePose
from agimus_sot.tools import _createOpPoint, assertEntityDoesNotExist, \
    matrixHomoInverse, matrixHomoProduct, entityIfMatrixHomo

class PreGrasp (Task):
    name_prefix = "pregrasp"
    meas_suffix = "_measured"

    ## Constructor
    # \param gripper object of type OpFrame
    # \param handle object of type OpFrame
    # \param otherGraspOnObject either None or a tuple (otherGripper, otherHandle)
    def __init__ (self, gripper, handle, otherGraspOnObject = None):
        super(PreGrasp, self).__init__()
        self.gripper = gripper
        self.handle = handle
        if otherGraspOnObject is not None:
            self.otherGripper = otherGraspOnObject[0]
            self.otherHandle  = otherGraspOnObject[1]
        else:
            self.otherGripper = None
            self.otherHandle  = None

    def makeTasks(self, sotrobot, withDerivative = False):
        assert self.gripper.enabled
        header = "{} / {}".format(self.gripper.name, self.handle.name)
        if self.otherGripper is not None:
            header += " - {} / {}".format(self.otherGripper.name, self.otherHandle.name)
        header += ":"

        self._makeAbsolute (sotrobot, withDerivative)

    ## Plug the position of linkName to \c outSignal.
    #  The pose of linkName must be computable by the SoT robot entity.
    def _plugRobotLink (self, sotrobot, linkName, poseSignal, Jsignal):
        plug(sotrobot.dynamic.signal(linkName), poseSignal)
        print("Plug robot link: no measument for " + linkName)
        if Jsignal is not None:
            plug(sotrobot.dynamic.signal("J"+linkName), Jsignal)

    ## Plug the position of linkName to \c outSignal.
    #  The pose of linkName is not computable by the SoT robot entity.
    #  \warning The topic linkName must have been created before.
    #  \todo reading the hpp joint topic sparsely will fail to provide the value
    #        at the expected time (the object pose will be asynchroneous with the
    #        rest of SoT).
    def _plugObjectLink (self, sotrobot, linkName, outSignal):
        linkNameMeas = linkName + self.meas_suffix

        # Create default value
        _createOpPoint (sotrobot, sotrobot.camera_frame)
        oMl = matrixHomoProduct(linkNameMeas + "_wrt_world",
                                sotrobot.dynamic.signal(sotrobot.camera_frame),
                                None,
                                check=False,)
        name = linkNameMeas + "wrt_world"
        if_ = entityIfMatrixHomo (name, condition=None,
                                  value_then=oMl.sout,
                                  value_else=None,
                                  check=False)
        self.addTfListenerTopic (linkNameMeas,
                                 frame0 = sotrobot.camera_frame,
                                 frame1 = linkNameMeas,
                                 signalGetters = [(oMl.sin(1), if_.condition),],
        )
        self.addHppJointTopic (linkName, signalGetters = [ if_.else_, ],)
        plug(if_.out, outSignal)

    ## Compute desired pose between gripper and handle.
    #  It is decomposed as \f$ jgMg^-1 * oMjg^-1 * oMlh * lhMh \f$.
    #  It creates the entity faMfbDes.
    #  Topic \c handle.fullLink must exists.
    def _referenceSignal (self, name, gripper, handle):
        # oMjg^-1 -> HPP joint
        self.oMjaDes_inv = matrixHomoInverse (name + "_oMjaDes_inv")
        self.addHppJointTopic (gripper.fullLink, signalGetters = [ self.oMjaDes_inv.sin, ],)
        # Plug it to FeaturePose
        self.faMfbDes = matrixHomoProduct (name + "_faMfbDes",
            gripper.lMf.inverse(), # jgMg^-1
            self.oMjaDes_inv.sout, # oMjg^-1 -> HPP joint
            None,                  # oMlh -> HPP joint
            handle.lMf,            # lhMh
            )
        # oMlh -> HPP joint
        self.extendSignalGetters(handle.fullLink, self.faMfbDes.sin(2))

    def _createTaskAndGain (self, name):
        # Create a task
        self.task = SotTask (name + "_task")
        self.task.add (self.feature.name)

        # Set the task gain
        self.gain = SafeGainAdaptive(name + "_gain")
        # See doc of SafeGainAdaptive to see how to plot the gain associated
        # to those values.
        self.gain.computeParameters(0.9,0.1,0.3,1.)
        plug(self.gain.gain, self.task.controlGain)
        plug(self.task.error, self.gain.error)

    ## \todo implement tracking of velocity
    def _makeAbsolute(self, sotrobot,
                      withDerivative):
        name = self._name(self.gripper.name, self.handle.fullName)

        assertEntityDoesNotExist(name+"_feature")
        self.feature = FeaturePose (name + "_feature")

        # Create the operational points
        _createOpPoint (sotrobot, self.gripper.link)

        self._plugRobotLink (sotrobot, self.gripper.link,
                self.feature.oMja, self.feature.jaJja)
        self.feature.jaMfa.value = self.gripper.lMf.homogeneous

        self.addHppJointTopic (self.handle.fullLink)
        self._plugObjectLink (sotrobot, self.handle.fullLink, self.feature.oMjb)
        self.feature.jbMfb.value = self.handle.lMf.homogeneous
        self.feature.jbJjb.value = np.zeros((6, sotrobot.dynamic.getDimension()))

        # Compute desired pose between gripper and handle.
        # Creates the entity faMfbDes
        self._referenceSignal (name, self.gripper, self.handle)
        plug(self.faMfbDes.sout, self.feature.faMfbDes)

        # Create a task and gain
        self._createTaskAndGain (name)

        if withDerivative:
            print("Relative pose constraint with derivative is not implemented yet.")
        self.task.setWithDerivative (False)

        self.tasks = [ self.task, ]

    def addVisualServoingTrace (self, tracer):
        from agimus_sot.tools import filename_escape
        self.addTrace(tracer)
        tracer.add (self.feature.name + ".faMfbDes", filename_escape(self.feature.name) + ".desired")
        tracer.add (self.feature.name + ".faMfb"   , filename_escape(self.feature.name) + ".actual")
