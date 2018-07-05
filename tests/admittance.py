from dynamic_graph import plug
from dynamic_graph.sot.core.integrator_euler import IntegratorEulerVectorMatrix
import dynamic_graph.sot.core as dgsc
import agimus_sot.control.controllers as controllers
import numpy as np
from math import sqrt, pi

import matplotlib
import matplotlib.pyplot as plt

### Parameters
# {{{

sign =  1
# sign = -1

# Torque amplitude
A_torque = sign * 5.

threshold_up   = A_torque / 2.,
threshold_down = 1e-5 * sign,
# threshold_up   = -1e8, # Up
# threshold_down =  1e8, # Down

## Measurement model (for simulation)
# M = 0.5
# M = 0.001
M = 0.
# d = 1.
d = 5.
k = 100.

# The angle at which a contact is created.
theta0 = sign * 1.
# theta0 = 40. * pi / 180.
# theta0 = 1. * pi / 180.
# theta0 = 0

## Estimated model (not used except est_theta0)
est_M = M
est_d = d
est_k = k
# Use for the initial position control. It should be greater than theta0
# so that position control will run until the object is touched.
est_theta0 = theta0 + sign * 10. * pi / 180.
# est_theta0 = theta0 + 1. * pi / 180.
# est_theta0 = theta0 - 10. * pi / 180.

# Time step and duration of simulation
dt = 0.001
N = int(2. / dt)
Nopen = int(1. / dt)
Nkeep_same_control = 0
# }}}

### Admittance controller
# {{{

from agimus_sot.control.gripper import AdmittanceControl

# z  = 0
# z  = sqrt(2)/2
z  = 1.
# z  = 5
# wn = 0.1
# wn = 1.
wn = 10.

# the control reaches a precision of 5% at
# z = 1: t = - log(0.05) / wn
# z < 1: t = - log(0.05 * sqrt(1-z**2)) / (z * wn),

print "Non-contact phase - PID:"
print " - z  =", z
print " - wn =", wn

# tau = 0.1
# tau = 1.
tau = 10.

# alpha_m, alpha_M = root_second_order(4*z**2 * est_M * k - d**2, 4*z**2*tau*k - 2*d, 1)
# alpha = alpha_m if alpha_m > 0 else alpha_M
# alpha = min (max(0.5, alpha), 1.5)
# alpha = 0.1
# alpha = 1.
alpha = 10.

print "Contact phase - PD:"
print " - tau   =", tau
# print " - alpha_m =", alpha_m
# print " - alpha_M =", alpha_M
print " - alpha =", alpha
# denom = alpha * M + tau
# print " - beta  1 =", alpha / denom
# print " - alpha 0 =", alpha * k / denom
# print " - alpha 1 =", (1+alpha*d)/denom
# print " - alpha 2 =", 1

admittance_controller = AdmittanceControl ("admittance",
        (0.,), (est_theta0,), (A_torque,), dt,
        threshold_up, threshold_down,
        wn = wn, z = z, alpha = alpha, tau = tau,)

# }}}

### Measurements (Feedback)
# {{{
admittance_controller.setupFeedbackSimulation(M, d, k, (theta0,))
# }}}

### Stability study
# {{{

# }}}

### Simulation
# {{{

t = 1
ts=[]
inputs=[]
conditions=[]
conditions_up=[]
conditions_down=[]
thetas=[]
phis=[]
omegas=[]
torques=[]

phi = - theta0
torque_control = False

theta = 0.

time_switch_to_torque   = -1000
time_switch_to_position = -1000

admittance_controller.setGripperClosed()
for i in range(1,N):
    if admittance_controller.sim_contact_condition.sout.value:
        if torque_control:
            print "Loosing contact at time", t
    if i == 1:
        measured_torque = (0,)
    else:
        measured_torque = admittance_controller.currentTorqueIn.value

    admittance_controller.switchEventToPositionCheck.recompute(t)
    admittance_controller.switchEventToTorqueCheck  .recompute(t)
    if t - time_switch_to_torque < Nkeep_same_control:
        admittance_controller.switch.latch.turnOn()
    if t - time_switch_to_position < Nkeep_same_control:
        admittance_controller.switch.latch.turnOff()
    admittance_controller.output.recompute(t);

    # TODO: I do not know whether this should be done or not.
    #       It lack a model of interaction when the contact is created...
    admittance_controller.phi2torque.output.recompute(t);

    if not torque_control and admittance_controller.switch.latch.out.value == 1:
        # Switch to torque control
        print "Switch to torque control at", t
        torque_control = True
        time_switch_to_torque = t
    elif torque_control and admittance_controller.switch.latch.out.value == 0:
        print "Switch to position control at", t
        torque_control = False
        time_switch_to_position = t

    ts     .append(t * dt)
    conditions.append(admittance_controller.switch.latch.out.value)
    conditions_up.append(admittance_controller.switch._condition_up.sout.value)
    conditions_down.append(admittance_controller.switch._condition_down.sout.value)
    try:
        inputs .append(admittance_controller.referenceTorqueIn.value[0])
    except IndexError:
        inputs .append(float("nan"))
    omegas .append(admittance_controller.omega2theta.reference.value[0])
    thetas .append(admittance_controller.omega2theta.output   .value[0])
    phis   .append(admittance_controller.theta2phi.sout   .value[0])
    torques.append(measured_torque)

    if t >= Nopen:
        admittance_controller.setGripperOpen ()
    t += 1

# }}}

### Plot the results
# {{{

skip=0

plt.subplot(3,1,1)
plt.plot(ts[skip:], inputs[skip:], 'b', ts[skip:], torques[skip:], 'g',
        )
plt.legend(["Reference torque", "Measured torque"
    ])
if sign>0:
    plt.gca().set_ylim(bottom=-A_torque,top=2*A_torque)
else:
    plt.gca().set_ylim(top=-A_torque,bottom=2*A_torque)
plt.grid()

plt.subplot(3,1,2)
plt.plot(ts[skip:], conditions_up[skip:], 'b-.',
        ts[skip:], conditions_down[skip:], 'g--',
        ts[skip:], conditions[skip:], 'r:',
        )
plt.legend(["Condition up", "Condition down"
    ,"Torque control active"
    ])
plt.gca().set_ylim(bottom=-0.1,top=1.1)
plt.grid()

plt.subplot(3,1,3)
plt.plot([ts[skip],ts[-1]], [theta0,theta0], "b",
         [ts[skip],ts[-1]], [est_theta0,est_theta0], "k--",
         ts[skip:], thetas[skip:], 'r',
         ts[skip:], phis  [skip:], 'm',
         ts[skip:], omegas[skip:], 'g +')
plt.legend(["theta_0", "Estimated theta_0", "Angle", "Velocity"])
plt.grid()

plt.tight_layout()
plt.show()

# }}}

# vim: set foldmethod=marker:
