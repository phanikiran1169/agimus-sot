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
