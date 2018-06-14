from dynamic_graph import plug
from dynamic_graph.sot.core.integrator_euler import IntegratorEulerVectorMatrix, IntegratorEulerVectorDouble
import dynamic_graph.sot.core as dgsc

class Controller:
    def __init__ (self, name, nums, denoms, period, initialValue):
        """
        - nums:   coeffs of the numerator in increasing order of derivatives
        - denoms: coeffs of the denominator in increasing order of derivatives
        """
        self.name = name
        if isinstance(nums[0], (float, int)):
            self.function = IntegratorEulerVectorDouble (name + "_transfer_function")
        else:
            self.function = IntegratorEulerVectorMatrix (name + "_transfer_function")
        for n in nums  : self.function.pushNumCoef   (n)
        for n in denoms: self.function.pushDenomCoef (n)

        self.function.sin.value = initialValue
        self.function.initialize()
        self.function.setSamplingPeriod(period)

        self.ref_m_meas = None

    def addFeedback (self):
        if self.ref_m_meas is None:
            self.ref_m_meas = dgsc.Add_of_vector(self.name + "_ref_m_meas")
            self.ref_m_meas.setCoeff1( 1)
            self.ref_m_meas.setCoeff2(-1)
            plug(self.ref_m_meas.sout, self.function.sin)

    @property
    def hasFeedback (self):
        return self.ref_m_meas is not None

    @property
    def reference (self):
        """ input signal """
        if self.ref_m_meas is not None:
            return self.ref_m_meas.sin1
        else:
            return self.function.sin

    @property
    def measurement (self):
        """ input signal """
        assert self.ref_m_meas is not None
        return self.ref_m_meas.sin2

    @property
    def output (self): return self.function.sout

    @property
    def outputDerivative (self): return self.function.derivativesout

def secondOrderOpenLoop (name, wn, z, period, initialValue):
    """
    Transfer function:
                    wn**2
    H(s) = -------------------------
           s**2 + 2*z*wn * s + wn**2

    - wm: corner frequency
    - z : damping
    """
    nums =   ( wn**2 ,)
    denoms = ( wn**2, 2*z*wn, 1. )
    return Controller (name, nums, denoms, period, initialValue)

def secondOrderClosedLoop (name, wn, z, period, initialValue):
    """
    Create a 2nd order transfer function with loop closure so that
    the overall relation between the input and the output is as below.
    It considers a unit feedback.
                    wn**2
    H(s) = -------------------------
           s**2 + 2*z*wn * s + wn**2

    - wm: corner frequency
    - z : damping
    """
    from math import sqrt
    wn_ol = wn / sqrt(2)
    z_ol  = z * sqrt(2)
    control = secondOrderOpenLoop (name, wn_ol, z_ol, period, initialValue)
    control.addFeedback()
    control.ref_m_meas.setCoeff1(2)
    return control
