from dynamic_graph import plug
from dynamic_graph.sot.core.integrator_euler import IntegratorEulerVectorMatrix
import dynamic_graph.sot.core as dgsc
import sot_hpp.control.controllers as controllers
import numpy as np
from math import sqrt, pi

import matplotlib
import matplotlib.pyplot as plt

### Parameters
# {{{

# Torque amplitude
A_torque = 5.

## Measurement model (for simulation)
M = 0.5
# M = 0.001
# M = 0.
d = 10.
k = 100.

# The angle at which a contact is created.
theta0 = 1.
# theta0 = 40. * pi / 180.
# theta0 = 1. * pi / 180.
# theta0 = 0

## Estimated model (not used except est_theta0)
est_M = M
est_d = d
est_k = k
# Use for the initial position control. It should be greater than theta0
# so that position control will run until the object is touched.
est_theta0 = theta0 + 10. * pi / 180.
# est_theta0 = theta0 + 1. * pi / 180.
# est_theta0 = theta0 - 10. * pi / 180.

# Time step and duration of simulation
dt = 0.001
# N = int(50 / dt)
# N = int(10 / dt)
N = int(1.5 / dt)
# N = int(0.5 / dt)
# N = 30
# }}}

### Admittance controller
# {{{

from sot_hpp.control.gripper import AdmittanceControl

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
tau = 1.
# tau = 10.

# alpha_m, alpha_M = root_second_order(4*z**2 * est_M * k - d**2, 4*z**2*tau*k - 2*d, 1)
# alpha = alpha_m if alpha_m > 0 else alpha_M
# alpha = min (max(0.5, alpha), 1.5)
# alpha = 0.1
alpha = 1.
# alpha = 10.

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
        (est_theta0,), (0.,), dt, (1e-5,),
        wn = wn, z = z, alpha = alpha, tau = tau,)

# }}}

### Measurements (Feedback)
# {{{
admittance_controller.setupFeedbackSimulation(M, d, k, (theta0,))
# }}}

### Input
# {{{

def sinus (t):
    from math import cos, sin, pi
    T = 0.5
    return (A_torque * (1. + 0.1 * sin(2 * pi / T * t)),)

def constant (t):
    return (A_torque,)

input = constant
# input = sinus
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
thetas=[]
omegas=[]
torques=[]

phi = - theta0
torque_control = False

theta = 0.

for i in range(1,N):
    input_torque = input(t * dt)
    if phi < 0:
        if torque_control:
            print "Loosing contact at time", t
        measured_torque = (0,)
    else:
        measured_torque = admittance_controller.phi2torque.output.value
        if len(measured_torque) != 1:
            print "Assuming very small measured torque at", t, phi
            measured_torque = (1e-4,)

    admittance_controller.referenceTorqueIn.value = input_torque

    admittance_controller.switchEventCheck.recompute(t)
    admittance_controller.output.recompute(t);

    # TODO: I do not know whether this should be done or not.
    #       It lack a model of interaction when the contact is created...
    admittance_controller.phi2torque.output.recompute(t);

    if not torque_control and admittance_controller.switch.latch.out.value == 1:
        # Switch to torque control
        print "Switch to torque control at", t
        torque_control = True
    elif torque_control and admittance_controller.switch.latch.out.value == 0:
        print "Switch to position control at", t
        torque_control = False

    ts     .append(t * dt)
    conditions.append(admittance_controller.switch.latch.out.value)
    inputs .append(input_torque)
    omegas .append(admittance_controller.omega2theta.reference.value[0])
    thetas .append(admittance_controller.omega2theta.output   .value[0])
    torques.append(measured_torque)

    theta = admittance_controller.omega2theta.output.value[0]
    phi = admittance_controller.theta2phi.sout.value[0]
    t += 1

# }}}

### Plot the results
# {{{

skip=0

plt.subplot(2,1,1)
plt.plot(ts[skip:], inputs[skip:], 'b', ts[skip:], torques[skip:], 'g',
        ts[skip:], conditions[skip:], 'r'
        )
plt.legend(["Reference torque", "Measured torque"
    ,"Torque control active"
    ])
plt.grid()

plt.subplot(2,1,2)
plt.plot([ts[skip],ts[-1]], [theta0,theta0], "b",
         [ts[skip],ts[-1]], [est_theta0,est_theta0], "k--",
         ts[skip:], thetas[skip:], 'r',
         ts[skip:], omegas[skip:], 'g +')
plt.legend(["theta_0", "Estimated theta_0", "Angle", "Velocity"])
plt.grid()

plt.show()

# }}}

# vim: set foldmethod=marker:
