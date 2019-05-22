from agimus_sot.srdf_parser import parse_srdf
from agimus_sot.control.gripper import AdmittanceControl
from agimus_sot.tools import OpFrame, EndEffector, Posture
from agimus_sot.factory import Affordance

from dynamic_graph.sot.core import SOT

dt = 0.001

desired_torque = (-5, )
threshold_up = tuple([ x / 10. for x in desired_torque ])
threshold_down = tuple([ x / 100. for x in desired_torque ])

srdfTalos = parse_srdf ("srdf/talos.srdf", packageName = "talos_data", prefix="talos")
srdfBox   = parse_srdf ("srdf/cobblestone.srdf", packageName = "gerard_bauzil", prefix="box")

gripperName = "talos/left_gripper"
handleName = "box/handle1"
gripperFrame = OpFrame(srdfTalos["grippers"][gripperName], robot.dynamic.model, True)
handleFrame  = OpFrame(srdfBox  ["handles" ][handleName ])

affordance = Affordance ("talos/left_gripper", "box/handle1",
        openControlType="position_torque", closeControlType="position_torque",
        # openControlType="position", closeControlType="position",
        refs = { "angle_open": (0,), "angle_close": (-0.5,), "torque": (-100.,) },
        simuParams = { "refPos": (-0.2,) }))

ee_task = EndEffector (robot, gripperFrame, "test")
ee_task.makeAdmittanceControl (affordance, "close", dt, simulateTorqueFeedback = True)

keep_posture = Posture ("posture_keep", robot)
keep_posture.tp.setWithDerivative (False)
keep_posture._signalPositionRef().value = robot.dynamic.position.value

def newSot(name):
    sot_keep = SOT (name)
    sot_keep.setSize(robot.dynamic.getDimension())
    sot_keep.damping.value = 0.001
    return name

sot_keep  = newSot ("keep" )
keep_posture.pushTo(sot_close)

# Use ee_task.ac.setGripperClosed() and ee_task.ac.setGripperOpen()
ee_task.setGripperOpen()
sot_ee  = newSot ("eecontrol" )
ee_task     .pushTo(sot_ee)
keep_posture.pushTo(sot_ee)

## Use two different tasks
# sot_open  = newSot ("open" )
# ee_open     .pushTo(sot_open)
# keep_posture.pushTo(sot_open)

# sot_close = newSot ("close")
# ee_close    .pushTo(sot_close)
# keep_posture.pushTo(sot_close)

def useSot(sot):
    from dynamic_graph import plug
    plug(sot.control, robot.device.control)
