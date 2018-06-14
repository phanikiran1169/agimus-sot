from dynamic_graph import plug
from dynamic_graph.sot.core.switch import SwitchVector
from dynamic_graph.sot.core.latch import Latch
from dynamic_graph.sot.core.event import Event
from dynamic_graph.sot.core.operator import CompareVector

class ControllerSwitch:
    def __init__ (self,name,controllers,threshold,):
        """
        Use controller 0 until the condition signal value becomes greater than threshold.
        Then use controller 1. Manually switch between controller using the latch.
        OFF means controller 0 and ON means controller 1.

        Currently support only two controllers.
        - controllers: output signal of a controller
        """
        # 1e-5 < measured_torque => torque control
        self._condition = CompareVector (name + "_condition")
        self._condition.setTrueIfAny(True)
        self._condition.sin1.value = threshold
        self._condition.sin2.value = threshold

        self._event = Event (name + "_event")
        self._latch = Latch(name + "_latch")
        self._latch.turnOff()

        self._switch = SwitchVector (name + "_switch")
        self._switch.setSignalNumber(len(controllers))

        plug(self._condition.sout, self._event.condition)
        plug(self._latch.out , self._switch.boolSelection)

        # This is necessary to initialize the event (the first recompute triggers the event...)
        self._event.check.recompute(0)
        self._event.addSignal (name + "_latch.turnOnSout")

        for n, sig in enumerate(controllers):
            plug(sig, self.signalIn(n))

    @property
    def measurement (self): return self._condition.sin2
    @property
    def threshold (self): return self._condition.sin1
    @property
    def threshold (self): return self._condition.sin1
    @property
    def signalOut (self): return self._switch.sout

    def signalIn (self, n): return self._switch.signal("sin" + str(n))

    @property
    def condition (self): return self._condition
    @property
    def event (self): return self._event
    @property
    def latch (self): return self._latch
    @property
    def switch (self): return self._switch
