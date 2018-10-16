from dynamic_graph.sot.core.switch import SwitchBoolean
from dynamic_graph.sot.core.event import Event
from dynamic_graph.ros import RosPublish
from agimus_sot.sot import Time
from dynamic_graph import plug

# TODO This should be removed when dynamic-graph-python
# provides the automatic convertion from expression to cascade of
# entities.
def logical_and_entity(name, inputs):
    from dynamic_graph.sot.core.operator import And
    a = And(name)
    a.setSignalNumber (len(inputs))
    for i,sig in enumerate(inputs): plug(sig, a.signal("sin"+str(i)))
    return a.sout
def logical_or_entity (name, inputs):
    from dynamic_graph.sot.core.operator import Or
    a = Or(name)
    a.setSignalNumber (len(inputs))
    for i,sig in enumerate(inputs): plug(sig, a.signal("sin"+str(i)))
    return a.sout
def norm_superior_to (name, input, thr):
    from dynamic_graph.sot.core.operator import Norm_of_vector, CompareDouble
    norm = Norm_of_vector (name + "_norm")
    plug (input, norm.sin)

    comparison = CompareDouble (name + "_comparison")
    comparison.sin1.value = thr
    plug (norm.sout, comparison.sin2)
    return norm, comparison
def norm_inferior_to (name, input, thr):
    from dynamic_graph.sot.core.operator import Norm_of_vector, CompareDouble
    norm = Norm_of_vector (name + "_norm")
    plug (input, norm.sin)

    comparison = CompareDouble (name + "_comparison")
    plug (norm.sout, comparison.sin1)
    comparison.sin2.value = thr
    return norm, comparison

class Events:
    def __init__ (self, name, robot):
        self.name = name

        # Setup entity that triggers the event.
        self.switch = SwitchBoolean (name + "_switch")
        self.event = Event (name + "_event")
        plug (self.switch.sout, self.event.condition)
        robot.device.after.addSignal (name + "_event.check")

        self.ros_publish = RosPublish (name + '_ros_publish')
        # self.ros_publish.add ('boolean', name, '/agimus/sot/event/' + name)
        # self.ros_publish.signal(name).value = int(True)
        self.ros_publish.add ('int', name, '/agimus/sot/event/' + name)

        self.event.addSignal (name + "_ros_publish.trigger")

    def getSignalNumber (self):
        return self.switch.getSignalNumber()

    def setSignalNumber (self, n):
        self.switch.setSignalNumber(n)

    def setSelectedSignal (self, n):
        self.switch.selection.value = n

    ## Creates entities to check whether the norm is
    ## superior to the threshold \c thr
    ##
    ## Use controlNormSignal to get the created signal.
    def setupNormOfControl (self, control, thr):
        self.norm, self.norm_comparision = norm_inferior_to (self.name+"_control",
                control, thr)

    ## Creates entities to check whether the norm is
    ## superior to the threshold \c thr
    ##
    ## Use timeSignal to get the created signal.
    def setupTime (self):
        self.time = Time (self.name + "_time")
        plug (self.time.now, self.ros_publish.signal(self.name))

    def setFutureTime (self, time):
        self.time.setTime (time)

    def conditionSignal (self, i):
        return self.switch.signal("sin"+str(i))

    @property
    def controlNormSignal (self):
        return self.norm_comparision.sout

    @property
    def timeEllapsedSignal (self):
        return self.time.after

    @property
    def remainsTimeSignal (self):
        return self.time.before
