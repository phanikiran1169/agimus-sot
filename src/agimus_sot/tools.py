# Copyright 2018 CNRS - Airbus SAS
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

def getTimerType (type):
    from dynamic_graph.sot.core.timer import TimerDouble, TimerMatrix, TimerMatrixHomo, TimerVector
    if type == "double":
        return TimerDouble
    elif type == "matrix":
        return TimerMatrix
    elif type == "matrixhomo":
        return TimerMatrixHomo
    elif type == "vector":
        return TimerVector
    else:
        raise ValueError ("Unknown type of timer.")

def filename_escape(value):
    """
    Normalizes string, converts to lowercase, removes non-alpha characters,
    and converts spaces to hyphens.
    """
    import unicodedata, re
    value = unicodedata.normalize('NFKD', unicode(value)).encode('ascii', 'ignore')
    value = unicode(re.sub('[^\w\s-]', '', value).strip().lower())
    value = unicode(re.sub('[-\s]+', '-', value))
    return str(value)

def insertTimerOnOutput (signal, type):
    """
    Plug the signal sout of the return entity instead of `signal` to
    input signal to enable the timer.
    - param signal an output signal.
    - return an Timer entity.
    """
    Timer = getTimerType (type)
    timer = Timer ("timer_of_" + signal.name)
    plug(signal, timer.sin)
    return timer

def insertTimer (signal, type):
    """
    - param signal a plugged input signal.
    - return an Timer entity.
    """
    assert signal.isPlugged()
    from dynamic_graph.sot.core.timer import TimerDouble, TimerMatrix, TimerMatrixHomo, TimerVector
    Timer = getTimerType (type)
    timer = Timer ("timer_of_" + signal.name)
    other = signal.getPlugged()
    plug(other, timer.sin)
    plug(timer.sout, signal)
    return timer

def parseHppName (hppjointname):
    if hppjointname == "universe": return "", "universe"
    return hppjointname.split('/', 1)

def transQuatToSE3 (p):
    from pinocchio import SE3, Quaternion
    from numpy import matrix
    return SE3 (Quaternion (p[6],p[3],p[4],p[5]).matrix(), matrix(p[0:3]).transpose())

def se3ToTuple (M):
    from dynamic_graph.sot.core.matrix_util import matrixToTuple
    return matrixToTuple (M.homogeneous)

def computeControlSelection (robot, joint_to_be_removed):
    pinmodel = robot.dynamic.model
    selection = ["1",] * pinmodel.nv
    for j in filter(lambda x: pinmodel.names[x.id] in joint_to_be_removed, pinmodel.joints[1:]):
        selection[j.idx_v:j.idx_v+j.nv] = ["0",] * j.nv
    selection.reverse()
    return "".join(selection)

def _createOpPoint (robot, name):
    if not robot.dynamic.hasSignal(name):
        robot.dynamic.createOpPoint(name, name)

def plugMatrixHomo(sigout, sigin):
    from dynamic_graph.signal_base import SignalBase
    from pinocchio import SE3
    if isinstance(sigout, tuple):
        sigin.value = sigout
    elif isinstance(sigout, SE3):
        sigin.value = se3ToTuple(sigout)
    elif isinstance(sigout, SignalBase):
        plug(sigout, sigin)

## \todo this should move to dynamic-graph-python
def assertEntityDoesNotExist(name):
    from dynamic_graph.entity import Entity
    assert name not in Entity.entities, "Entity " + name + " already exists."

def matrixHomoProduct(name, *args, **kwargs):
    from dynamic_graph.sot.core.operator import Multiply_of_matrixHomo
    if kwargs.get('check',True): assertEntityDoesNotExist(name)
    ent = Multiply_of_matrixHomo (name)
    ent.setSignalNumber(len(args))
    for i, valueOrSignal in enumerate(args):
        if valueOrSignal is None: continue
        plugMatrixHomo (valueOrSignal, ent.signal('sin'+str(i)))
    return ent

def matrixHomoInverse(name, valueOrSignal=None, check=True):
    from dynamic_graph.sot.core.operator import Inverse_of_matrixHomo
    if check: assertEntityDoesNotExist(name)
    ent = Inverse_of_matrixHomo (name)
    plugMatrixHomo(valueOrSignal, ent.sin)
    return ent
