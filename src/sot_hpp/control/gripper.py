from dynamic_graph import plug

class AdmittanceControl:
    """
    Encapsulate two controllers: a position controller and a torque controller.
    The position controller is used to create a contact.
    The torque controller is then use to maintain a desired force.
    Both controller outputs a velocity command to be sent to entity Device.
    """
    def __init__ (self, name, estimated_theta0, desired_torque, period, threshold,
            wn = 10., z = 1., alpha = 1., tau = 1.,
            ):
        """
        - estimated_theta0: Use for the initial position control. It should correspond to a configuration in collision.
                            The closer to contact configuration, the least the overshoot.
        - desired_torque: The torque to be applied on the object.
        - period: The SoT integration time.
        - threshold: When one component of the torque becomes greater that threshold, switch to torque control
        - wn, z: corner frequency and damping of the second order position control.
        - alpha, tau: amplitude and time constant of the first order torque control.
        """
        self.name = name
        self.est_theta0 = estimated_theta0
        self.desired_torque = desired_torque
        self.threshold = threshold
        self.dt = period

        self._makePositionControl (wn, z)

        self._makeTorqueControl (alpha, tau)

        self._makeControllerSwich ()

    def resetToPositionControl (self):
        self.switch.latch.turnOff()

    ### Feed-forward - non-contact phase
    def _makePositionControl (self, wn, z):
        """
        the control reaches a precision of 5% at
        * z = 1: t = - log(0.05) / wn
        * z < 1: t = - log(0.05 * sqrt(1-z**2)) / (z * wn),
        """
        from sot_hpp.control.controllers import secondOrderClosedLoop
        self.position_controller = secondOrderClosedLoop (self.name + "_position", wn, z, self.dt, [0. for _ in self.est_theta0])
        self.position_controller.reference.value = self.est_theta0

    ### Feed-forward - contact phase
    def _makeTorqueControl (self, alpha, tau):
        from sot_hpp.control.controllers import Controller
        self.torque_controller = Controller (self.name + "_torque", (alpha,), (1., tau), self.dt, [0. for _ in self.est_theta0])
        self.torque_controller.addFeedback()
        self.torque_controller.reference.value = self.desired_torque

    ### Setup switch between the two control scheme
    def _makeControllerSwich (self):
        from sot_hpp.control.switch import ControllerSwitch

        self.switch = ControllerSwitch (self.name + "_switch",
                (self.position_controller.outputDerivative, self.torque_controller.output),
                self.threshold)

    ### Setup event to tell when object is grasped
    def _makeSteadyControlEvent (self):
        #TODO
        # Either check:
        #  - the torque error (and its derivatives ?)
        #  - the output velocity command (and its derivatives ?)
        # Should we use a time delay to check whether it is steady ?
        pass

    ### Setup event to tell when object is grasped
    def setupFeedbackSimulation (self, mass, damping, spring, theta0):
        from sot_hpp.control.controllers import Controller
        from dynamic_graph.sot.core import Add_of_vector
        from dynamic_graph_hpp.sot import DelayVector

        ## omega -> theta
        self.omega2theta = Controller (self.name + "_sim_omega2theta",
                (1.,), (0., 1.), self.dt, [0. for _ in self.est_theta0])
        plug (self.output, self.omega2theta.reference)

        delayTheta = DelayVector (self.name + "_sim_theta_delay")
        delayTheta.setMemory (tuple([0. for _ in self.est_theta0]))
        plug (self.omega2theta.output, delayTheta.sin)
        plug (delayTheta.previous, self.currentPositionIn)

        ## theta -> phi = theta - theta0
        self.theta2phi = Add_of_vector(self.name + "_sim_theta2phi")
        self.theta2phi.setCoeff1 ( 1)
        self.theta2phi.setCoeff2 (-1)
        plug (delayTheta.current, self.theta2phi.sin1)
        self.theta2phi.sin2.value = theta0

        ## phi -> torque
        from dynamic_graph.sot.core.switch import SwitchVector
        from dynamic_graph.sot.core.operator import CompareVector
        self.sim_contact_condition = CompareVector(self.name + "_sim_contact_condition")

        plug (self.theta2phi.sout, self.sim_contact_condition.sin1)
        self.sim_contact_condition.setTrueIfAny(False)

        self.sim_switch = SwitchVector (self.name + "_sim_torque")
        self.sim_switch.setSignalNumber(2)
        plug (self.sim_contact_condition.sout, self.sim_switch.boolSelection)

        # Non contact phase
        self.sim_contact_condition.sin2.value = [0. for _ in self.est_theta0]
        # Contact phase
        self.phi2torque = Controller (self.name + "_sim_phi2torque",
                (spring, damping, mass,), (1.,),
                self.dt, [0. for _ in self.est_theta0])
        #TODO if M != 0: phi2torque.pushNumCoef(((M,),))
        plug (self.theta2phi.sout, self.phi2torque.reference)

        # Condition
        # if phi < 0 -> no contact -> torque = 0
        self.sim_switch.sin1.value = [0. for _ in self.est_theta0]
        # else       ->    contact -> phi2torque
        plug (self.phi2torque.output, self.sim_switch.sin0)

        delay = DelayVector (self.name + "_sim_torque_delay")
        delay.setMemory (tuple([0. for _ in self.est_theta0]))
        # plug (self.phi2torque.output, delay.sin)
        plug (self.sim_switch.sout, delay.sin)
        plug (delay.current , self.currentConditionIn)
        plug (delay.previous, self.currentTorqueIn)

    @property
    def output (self):
        return self.switch.signalOut

    @property
    def referencePositionIn (self):
        return self.position_controller.reference

    @property
    def referenceTorqueIn (self):
        return self.torque_controller.reference

    @property
    def currentPositionIn (self):
        return self.position_controller.measurement

    @property
    def currentTorqueIn (self):
        return self.torque_controller.measurement

    @property
    def currentConditionIn (self):
        return self.switch.measurement

    @property
    def switchEventCheck (self):
        return self.switch.event.check

# vim: set foldmethod=indent
