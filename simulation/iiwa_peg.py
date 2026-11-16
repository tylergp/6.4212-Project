"""IIWA 7 + wrist-mounted peg + chamfered insertion fixture in Drake.

Run from your project with its virtual environment activated:
    python iiwa_peg_scene.py
For a short check without Meshcat:
    python iiwa_peg_scene.py --headless --duration 0.1

Dependencies: drake, manipulation, numpy (Python 3.13 recommended).
This starter holds an initial pose; it does not implement insertion control.
The fixture is the textbook's open-sided planar slot, not an enclosed socket.
The arm model has no arm collision geometry; peg/fixture/table contact is enabled.

Model and scene references (assets are loaded from installed packages):
https://github.com/RussTedrake/manipulation/blob/master/book/force/exercises/04_peg_in_hole.ipynb
https://github.com/tlpmit/6.4210-2-fall26-handouts/blob/main/ps2/pick_and_place.py
"""

import argparse

import numpy as np
from manipulation.station import LoadScenario, MakeHardwareStation
from pydrake.all import (
    InverseKinematics,
    RotationMatrix,
    Simulator,
    Solve,
    StartMeshcat,
)


# Units: meters and radians. The table top and robot base are at world z = 0.
# The fixture mesh is offset in its own y direction, so y = 0.09 centers its
# insertion channel on world y = 0. Its top is approximately z = 0.165 m.
HOLE_ORIGIN = np.array([0.55, 0.09, 0.0])
PEG_TIP_TARGET = np.array([0.5485, 0.0, 0.23])
Q_SEED = np.array([0.0, 0.4, 0.0, -1.6, 0.0, 1.2, 0.0])

SCENARIO = f"""
plant_config:
    time_step: 0.001
    contact_model: hydroelastic_with_fallback
    discrete_contact_approximation: sap

directives:
- add_model:
    name: iiwa
    file: package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf
    default_joint_positions:
        iiwa_joint_1: [0.0]
        iiwa_joint_2: [0.4]
        iiwa_joint_3: [0.0]
        iiwa_joint_4: [-1.6]
        iiwa_joint_5: [0.0]
        iiwa_joint_6: [1.2]
        iiwa_joint_7: [0.0]
- add_weld:
    parent: world
    child: iiwa::iiwa_link_0

- add_model:
    name: table
    file: package://manipulation/hydro/extra_heavy_duty_table_surface_only_collision.sdf
- add_weld:
    parent: world
    child: table::link
    X_PC:
        translation: [0.5, 0.0, -0.7645]

- add_model:
    name: peg
    file: package://manipulation/peg_in_hole/peg.urdf
- add_weld:
    parent: iiwa::iiwa_link_7
    child: peg::peg_body_link
    X_PC:
        translation: [0.0, 0.0, 0.10]
        rotation: !Rpy {{ deg: [0.0, 180.0, 0.0] }}

- add_model:
    name: hole
    file: package://manipulation/peg_in_hole/hole_chamfered.sdf
- add_weld:
    parent: world
    child: hole::hole_chamfered_body_link
    X_PC:
        translation: [{HOLE_ORIGIN[0]}, {HOLE_ORIGIN[1]}, {HOLE_ORIGIN[2]}]

model_drivers:
    iiwa: !IiwaDriver
        control_mode: position_only
        hand_model_name: peg
"""

def initial_configuration(plant, plant_context):
    """Place the peg vertically above the slot using a one-time IK solve."""
    peg = plant.GetModelInstanceByName("peg")
    peg_frame = plant.GetFrameByName("peg_body_link", peg)
    ik = InverseKinematics(plant, plant_context)

    # The box is 0.20 m long. With its local z pointing up, the bottom tip is
    # at local z = -0.10 m. This leaves about 6.5 cm above the fixture entrance.
    ik.AddPositionConstraint(
        peg_frame,
        [0.0, 0.0, -0.10],
        plant.world_frame(),
        PEG_TIP_TARGET - 0.001,
        PEG_TIP_TARGET + 0.001,
    )
    ik.AddOrientationConstraint(
        plant.world_frame(), RotationMatrix(),
        peg_frame, RotationMatrix(), 0.01,
    )
    prog = ik.prog()
    prog.AddQuadraticErrorCost(np.eye(7), Q_SEED, ik.q())
    prog.SetInitialGuess(ik.q(), Q_SEED)
    result = Solve(prog)
    if not result.is_success():
        raise RuntimeError(
            "Could not find the initial robot pose. If you moved the fixture, "
            "adjust PEG_TIP_TARGET and Q_SEED to a reachable pose."
        )
    return result.GetSolution(ik.q())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Disable Meshcat.")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Seconds to simulate before holding the viewer open.")
    args = parser.parse_args()
    if args.duration < 0 or not np.isfinite(args.duration):
        parser.error("--duration must be a finite nonnegative number")

    meshcat = None if args.headless else StartMeshcat()
    scenario = LoadScenario(data=SCENARIO)
    station = MakeHardwareStation(scenario, meshcat=meshcat, hardware=False)
    plant = station.GetSubsystemByName("plant")
    simulator = Simulator(station)
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyMutableContextFromRoot(context)

    iiwa = plant.GetModelInstanceByName("iiwa")
    q_hold = initial_configuration(plant, plant_context)
    plant.SetPositions(plant_context, iiwa, q_hold)
    plant.SetVelocities(plant_context, iiwa, np.zeros(7))
    station.GetInputPort("iiwa.position").FixValue(context, q_hold)

    simulator.set_target_realtime_rate(0.0 if args.headless else 1.0)
    simulator.Initialize()
    station.ForcedPublish(context)
    if meshcat is not None:
        print(f"Open Meshcat in your browser: {meshcat.web_url()}")
    print("Scene: IIWA 7, wrist-mounted peg, chamfered fixture, and table.")
    print("Holding the starting pose; insertion control is not implemented.")
    simulator.AdvanceTo(args.duration)

    q_final = plant.GetPositions(plant_context, iiwa)
    if not np.all(np.isfinite(q_final)):
        raise RuntimeError("Simulation produced nonfinite joint positions.")
