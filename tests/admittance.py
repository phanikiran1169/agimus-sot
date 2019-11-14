from __future__ import print_function
from dynamic_graph import plug
from dynamic_graph.sot.core.integrator_euler import IntegratorEulerVectorMatrix
import dynamic_graph.sot.core as dgsc
import agimus_sot.control.controllers as controllers
import numpy as np, sys
from math import sqrt, pi

import matplotlib
import matplotlib.pyplot as plt

### Parameters
# {{{

# sign =  1
sign = -1

# Torque amplitude
#desired_torque = (sign * 5.,)
desired_torque = (sign * 0.07,)

#threshold_up   = desired_torque[0] / 2.,
#threshold_down = 1e-5 * sign,

threshold_up = tuple([ x / 10. for x in desired_torque ])
threshold_down = tuple([ x / 100. for x in desired_torque ])
# threshold_up   = -1e8, # Up
# threshold_down =  1e8, # Down

## Measurement model (for simulation)
M = 0.
d = 5.
k = 100.

# Torque control model
nums_tor   = 1.,
denoms_tor = 1., 1.,

# The angle at which a contact is created.
#theta0 = sign * 1.
theta0 = sign * 0.4
# theta0 = 40. * pi / 180.
# theta0 = 1. * pi / 180.
# theta0 = 0

## Estimated model (not used except est_theta0)
est_M = M
est_d = d
est_k = k
# Use for the initial position control. It should be greater than theta0
# so that position control will run until the object is touched.
#est_theta0 = theta0 + sign * 10. * pi / 180.
est_theta0 = -0.5
# est_theta0 = theta0 + 1. * pi / 180.
# est_theta0 = theta0 - 10. * pi / 180.

# Time step and duration of simulation
dt = 0.001
N = int(2. / dt)
#Nopen = int(1. / dt)
Nkeep_same_control = 0
# }}}

### Admittance controller
# {{{

from agimus_sot.control.gripper import PositionAndAdmittanceControl

z  = 1.
wn = 10.
#nums_tor   = (0.0, 0.01,)
#denoms_tor = (1.,)

print ("Non-contact phase - PID:")
print (" - z  =", z)
print (" - wn =", wn)


print ("Contact phase - PD:")
print (" - nums   =", nums_tor)
print (" - denoms =", denoms_tor)

admittance_controller = PositionAndAdmittanceControl ("admittance",
        (0.,), (est_theta0,), desired_torque, dt,
        threshold_up, threshold_down,
        wn = wn, z = z,
        nums_tor   = nums_tor,
        denoms_tor = denoms_tor,
        )

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
contacts=[]

phi = - theta0
torque_control = False

theta = 0.
omega = 0.
max_acc = 2e2

time_switch_to_torque   = -1000
time_switch_to_position = -1000

for i in range(1,N):
    # Set current position
    admittance_controller.omega2theta.sin1.value = theta,
    admittance_controller.omega2theta.sin1.time = t
    admittance_controller._sim_theta2phi.sin1.value = theta,
    admittance_controller._sim_theta2phi.sin1.time = t
    admittance_controller.position_controller.measurement.value = theta,
    admittance_controller.position_controller.measurement.time = t

    admittance_controller.sim_contact_condition.sout.recompute(t)
    if admittance_controller.sim_contact_condition.sout.value == 1 and torque_control:
        print ("Loosing contact at time", t)
        #break
        #sys.exit(0)
    #if torque_control:
        #break

    #admittance_controller.sim_contact_condition.sout.recompute(t)
    #admittance_controller._sim_torque.current.recompute(t)
    admittance_controller.switchEventToPositionCheck.recompute(t)
    admittance_controller.switchEventToTorqueCheck  .recompute(t)
    if i == 1:
        measured_torque = (0,)
    else:
        admittance_controller.currentTorqueIn.recompute(t)
        measured_torque = admittance_controller.currentTorqueIn.value

    #if t - time_switch_to_torque < Nkeep_same_control:
    #    admittance_controller.switch.latch.turnOn()
    #if t - time_switch_to_position < Nkeep_same_control:
    #    admittance_controller.switch.latch.turnOff()
    admittance_controller.outputPosition.recompute(t);

    # TODO: I do not know whether this should be done or not.
    #       It lack a model of interaction when the contact is created...
    #admittance_controller.phi2torque.output.recompute(t);

    if not torque_control and admittance_controller.switch.latch.out.value == 1:
        # Switch to torque control
        print ("Switch to torque control at", t)
        torque_control = True
        time_switch_to_torque = t
    elif torque_control and admittance_controller.switch.latch.out.value == 0:
        print ("Switch to position control at", t)
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
    omega = (admittance_controller.outputPosition.value[0]-theta)/dt

    omegas .append(omega)
    thetas .append(admittance_controller.outputPosition.value[0])
    phis   .append(admittance_controller._sim_theta2phi.sout.value[0])
    torques.append(measured_torque[0])
    contacts.append(admittance_controller.sim_contact_condition.sout.value)

    ## Bound accelerations to simulate mechanics
    #acc_ang = (omega_cmd - omega)/dt
    #if acc_ang != acc_ang:
    #    break
    #if abs(acc_ang) < max_acc:
    #    omega = omega_cmd
    #else:
    #    acc_ang_sat = max_acc * (1 if acc_ang>=0 else -1)
    #    print("Acceleration saturated", acc_ang, omega, omega+dt*acc_ang_sat)
    #    omega = omega + dt*acc_ang_sat
    #    if time_switch_to_torque+Nkeep_same_control >= t and omega * sign < 0:
    #        omega = 0
    #        print("Force torque control")
    #    elif time_switch_to_position+Nkeep_same_control >= t and omega * sign > 0:
    #        omega = 0
    #        print("Force position control")
    theta += dt * omega

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
    plt.gca().set_ylim(bottom=-desired_torque[0],top=2*desired_torque[0])
else:
    plt.gca().set_ylim(top=-desired_torque[0],bottom=2*desired_torque[0])
plt.grid()

plt.subplot(3,1,2)
plt.plot(ts[skip:], conditions_up[skip:], 'b-.',
        ts[skip:], conditions_down[skip:], 'g--',
        ts[skip:], conditions[skip:], 'r:',
        ts[skip:], contacts[skip:], 'k:',
        )
plt.legend(["Condition up", "Condition down"
    ,"Torque control active"
    ,'contact'
    ])
plt.gca().set_ylim(bottom=-0.1,top=1.1)
plt.grid()

plt.subplot(3,1,3)
plt.plot([ts[skip],ts[-1]], [theta0,theta0], "b",
         [ts[skip],ts[-1]], [est_theta0,est_theta0], "k--",
         ts[skip:], thetas[skip:], 'r',
         #ts[skip:], phis  [skip:], 'm',
         #ts[skip:], omegas[skip:], 'g +',
         )
plt.legend(["theta_0", "Estimated theta_0", "Angle", "Velocity"])
plt.grid()

plt.tight_layout()
plt.show()

# }}}

# vim: set foldmethod=marker:
