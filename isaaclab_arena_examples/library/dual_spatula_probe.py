# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Test two sliding spatula jaws using contact friction without book attachments."""

import argparse
import hashlib
import json
import math
import os
import re
import sys
from pathlib import Path


def parse_args(return_workflow=False):
    """Return the geometry, control and simulation options."""
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--angles", nargs="+", type=float, default=[0, 10, -10, 20, -20])
    parser.add_argument("--layout", choices=["roomy", "crowded"], default="roomy")
    parser.add_argument("--mode", choices=["slide", "extract"], default="slide")
    parser.add_argument("--jaw-force", type=float, default=4)
    parser.add_argument("--gravity-compensation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--friction", type=float, default=0.5)
    parser.add_argument("--dt", type=float, default=1 / 240)
    parser.add_argument("--s1", action="store_true")
    parser.add_argument("--stop-after", help="Save a diagnostic capture after a phase without declaring success.")
    parser.add_argument(
        "--return-book", action="store_true", help="Clear the shelf, then physically shelve a staged return book."
    )
    parser.add_argument(
        "--s1-self-collisions",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable S1 self contact; the supplied collision meshes overlap at nominal poses.",
    )
    parser.add_argument(
        "--probe-init",
        action="store_true",
        help="Record initial contact constraints for 0.2 seconds only.",
    )
    parser.add_argument(
        "--external-forces-per-iteration",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    AppLauncher.add_app_launcher_args(parser)
    if return_workflow:
        parser.set_defaults(s1=True, return_book=True, mode="extract", angles=[-10], layout="crowded")
    args = parser.parse_args()
    assert args.device == "cpu" and args.dt > 0
    assert args.jaw_force > 0 and args.friction > 0
    assert not args.s1 or len(args.angles) == 1, "S1 mode supports a single shelf per run."
    assert not args.return_book or (args.s1 and args.mode == "extract"), "Return workflow requires S1 extraction mode."
    assert os.geteuid() != 0, "Run physics as a non-root user."
    assert not args.output_dir.exists() or not any(args.output_dir.iterdir()), "Use a fresh output directory."
    args.headless, args.visualizer = True, []
    return args


def make_case(sim, args, index, angle):
    """Create three dynamic books and a floating carriage with two driven prismatic jaws.

    Args:
        sim: Simulation context before reset.
        args: Parsed experiment options.
        index: Independent shelf index.
        angle: Initial target book roll in degrees.

    Returns:
        Case state and its three book objects.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObject, RigidObjectCfg

    cy = index * 0.8
    root = f"/World/Case{index}"
    material = sim_utils.RigidBodyMaterialCfg(
        static_friction=args.friction,
        dynamic_friction=args.friction * 0.8,
        restitution=0,
    )

    def box(path, size, xyz, color, mass=None, rot=(0, 0, 0, 1)):
        cfg = sim_utils.CuboidCfg(
            size=size,
            physics_material=material,
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.0003, rest_offset=0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
            rigid_props=(
                sim_utils.RigidBodyPropertiesCfg(
                    solver_position_iteration_count=32,
                    solver_velocity_iteration_count=8,
                    max_linear_velocity=2,
                    max_angular_velocity=10,
                )
                if mass is not None
                else None
            ),
            mass_props=(sim_utils.MassPropertiesCfg(mass=mass) if mass is not None else None),
            activate_contact_sensors=mass is not None,
        )
        cfg.func(path, cfg, translation=tuple(xyz), orientation=rot)

    box(root + "/Shelf", (0.8, 0.52, 0.04), (-0.10, cy, 0.84), (0.4, 0.27, 0.16))
    for sign in (-1, 1):
        box(
            root + f"/End{sign+1}",
            (0.20, 0.012, 0.26),
            (0, cy + sign * 0.22, 0.99),
            (0.3, 0.33, 0.35),
        )
    theta = math.radians(angle)
    half_y = (0.035 * abs(math.cos(theta)) + 0.22 * abs(math.sin(theta))) / 2
    books = []
    for j in (-1, 0, 1):
        roll = theta if j == 0 else 0
        gap = 0.06 if j == -1 and args.layout == "roomy" else 0.006
        y = cy + j * (half_y + 0.0175 + gap)
        z = 0.86 + (0.22 * abs(math.cos(roll)) + 0.035 * abs(math.sin(roll))) / 2 + 0.001
        path = root + f"/Book{j+1}"
        rot = tuple(Rotation.from_euler("x", roll).as_quat())
        box(
            path,
            (0.16, 0.035, 0.22),
            (0, y, z),
            (0.65, 0.12, 0.09) if j == 0 else (0.15, 0.4, 0.28),
            0.18,
            rot,
        )
        books.append(RigidObject(RigidObjectCfg(prim_path=path, spawn=None)))
    if args.return_book:
        path = root + "/ReturnBook"
        box(path, (0.16, 0.035, 0.22), (0, cy + 0.14, 0.971), (0.12, 0.26, 0.72), 0.18)
        books.append(RigidObject(RigidObjectCfg(prim_path=path, spawn=None)))
    tool_root = root + "/Tool"
    start = np.array([-0.42, cy, 0.98])
    tool, sensors = make_dual_tool(
        sim, args, tool_root, start, [books[1].cfg.prim_path] + ([books[3].cfg.prim_path] if args.return_book else [])
    )
    case = dict(
        index=index,
        angle_deg=angle,
        cy=cy,
        books=books,
        tool=tool,
        sensors=sensors,
        goal=start.copy(),
        rotation=Rotation.identity(),
        opening=0.0205,
        trace=[],
        baseline=None,
        peak_normal_force=0,
        initial=start,
    )
    return case, books


def make_dual_tool(sim, args, tool_root, start, target_paths):
    """Create the verified force-limited two-blade articulation at a wrist mount.

    Args:
        sim: Simulation before reset.
        args: Options with friction and jaw_force.
        tool_root: Unique USD tool root.
        start: Initial carriage position.
        target_paths: Book paths for separate contact measurements.

    Returns:
        Tool articulation and its two contact sensors.
    """
    import numpy as np

    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.sensors import ContactSensor, ContactSensorCfg
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    start = np.asarray(start)
    material = sim_utils.RigidBodyMaterialCfg(
        static_friction=args.friction,
        dynamic_friction=args.friction * 0.8,
        restitution=0,
    )

    def box(path, size, xyz, color, mass=None, rot=(0, 0, 0, 1)):
        cfg = sim_utils.CuboidCfg(
            size=size,
            physics_material=material,
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.0003, rest_offset=0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
            rigid_props=(
                sim_utils.RigidBodyPropertiesCfg(
                    solver_position_iteration_count=32,
                    solver_velocity_iteration_count=8,
                    max_linear_velocity=2,
                    max_angular_velocity=10,
                )
                if mass is not None
                else None
            ),
            mass_props=(sim_utils.MassPropertiesCfg(mass=mass) if mass is not None else None),
            activate_contact_sensors=mass is not None,
        )
        cfg.func(path, cfg, translation=tuple(xyz), orientation=rot)

    prim = UsdGeom.Xform.Define(sim.stage, tool_root).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr(True)
    box(tool_root + "/Carrier", (0.04, 0.13, 0.04), start, (0.32, 0.38, 0.43), 0.26)
    sensors = []
    for side, sign in (("Left", -1), ("Right", 1)):
        jaw = tool_root + "/" + side
        box(
            jaw,
            (0.12, 0.003, 0.08),
            start + [0.20, sign * 0.0205, 0],
            (0.55, 0.65, 0.75),
            0.08,
        )
        # Narrow shanks connect both blades to the sliding guides outside the shelf.
        box(jaw + "/Shank", (0.12, 0.003, 0.025), (-0.12, 0, 0), (0.45, 0.52, 0.58))
        joint = UsdPhysics.PrismaticJoint.Define(sim.stage, tool_root + "/jaw_" + side.lower())
        joint.CreateBody0Rel().SetTargets([tool_root + "/Carrier"])
        joint.CreateBody1Rel().SetTargets([jaw])
        joint.CreateAxisAttr("Y")
        joint.CreateLocalPos0Attr(Gf.Vec3f(0.20, 0, 0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0))
        joint.CreateLocalRot0Attr(Gf.Quatf(1))
        joint.CreateLocalRot1Attr(Gf.Quatf(1))
        joint.CreateLowerLimitAttr(-0.06 if sign < 0 else 0.015)
        joint.CreateUpperLimitAttr(-0.015 if sign < 0 else 0.06)
        joint.CreateCollisionEnabledAttr(False)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(1500)
        drive.CreateDampingAttr(15)
        drive.CreateMaxForceAttr(args.jaw_force)
        drive.CreateTargetPositionAttr(sign * 0.0205)
        sensor = ContactSensor(
            ContactSensorCfg(
                prim_path=jaw,
                update_period=0,
                filter_prim_paths_expr=target_paths,
            )
        )
        sensors.append(sensor)
    cfg = ArticulationCfg(
        prim_path=tool_root,
        spawn=None,
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(start), joint_pos={"jaw_left": -0.0205, "jaw_right": 0.0205}
        ),
        actuators={
            "jaws": ImplicitActuatorCfg(
                joint_names_expr=["jaw_.*"],
                stiffness=1500,
                damping=15,
                effort_limit_sim=args.jaw_force,
                velocity_limit_sim=0.015,
            )
        },
    )
    tool = Articulation(cfg)
    return tool, sensors


def pose(body):
    """Return a rigid object's measured XYZ/XYZW pose."""
    return body.data.root_link_pose_w.torch[0].numpy().copy()


def shelf_free_slot(poses, preferred_y, min_width=0.063):
    """Return an unoccupied interval between measured book bounds nearest the requested slot.

    Args:
        poses: XYZ/XYZW poses of all books already on the shelf.
        preferred_y: Requested lateral position, in metres.
        min_width: Minimum gap including the opened blades and clearance.

    Returns:
        Lower and upper Y bounds of the selected free interval.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    bounds = []
    for p in poses:
        half_y = float(np.abs(Rotation.from_quat(p[3:]).as_matrix()[1]) @ [0.08, 0.0175, 0.11])
        bounds.append((float(p[1]) - half_y, float(p[1]) + half_y))
    bounds.sort()
    occupied = []
    for lower, upper in bounds:
        if occupied and lower <= occupied[-1][1]:
            occupied[-1] = (occupied[-1][0], max(occupied[-1][1], upper))
        else:
            occupied.append((lower, upper))
    intervals = [(left[1], right[0]) for left, right in zip(occupied, occupied[1:]) if right[0] - left[1] >= min_width]
    assert intervals, f"No shelf gap fits the opened blades with clearance: {bounds}"
    return min(intervals, key=lambda gap: abs((gap[0] + gap[1]) / 2 - preferred_y))


def mount_s1(sim, case, args):
    """Connect the sliding-jaw carriage to S1's right wrist before physics starts.

    Args:
        sim: Simulation before reset.
        case: Single physical gripper case.
        args: Experiment options including S1 self-contact settings.

    Returns:
        Robot, disabled mounting joint, kinematics and chassis position.
    """
    import numpy as np

    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation
    from pxr import Gf, UsdPhysics

    from isaaclab_arena_examples.library.galbot_s1_robot import make_galbot_cfg
    from isaaclab_arena_examples.library.s1_library_ik import S1Kinematics
    from isaaclab_arena_examples.library.spatula_contact_probe import spawn_tool_robot

    base = np.array([-1.08, 0.20, 0])
    ik = S1Kinematics()
    cfg = make_galbot_cfg(position=tuple(base))
    cfg.spawn.func = spawn_tool_robot
    cfg.spawn.rigid_props.disable_gravity = False
    cfg.spawn.activate_contact_sensors = True
    cfg.spawn.articulation_props.enabled_self_collisions = args.s1_self_collisions
    cfg.actuators.pop("hands")
    cfg.actuators["stock_left_gripper"] = ImplicitActuatorCfg(
        joint_names_expr=["left_(active|passive)_joint.*"], stiffness=20, damping=1
    )
    for group in cfg.actuators.values():
        matching = {
            n: j
            for n, j in ik.by_name.items()
            if j.find("limit") is not None and any(re.fullmatch(pattern, n) for pattern in group.joint_names_expr)
        }
        group.effort_limit_sim = {n: float(j.find("limit").get("effort")) for n, j in matching.items()}
        group.velocity_limit_sim = {n: float(j.find("limit").get("velocity")) for n, j in matching.items()}
    seed = [cfg.init_state.joint_pos[f"right_arm_joint{i}"] for i in range(1, 8)]
    q, _ = ik.solve("right", case["initial"] - [0.15, 0, 0] - base, 0.25, seed)
    cfg.init_state.joint_pos.update({f"right_arm_joint{i}": float(v) for i, v in enumerate(q, 1)})
    robot = Articulation(cfg)
    joint = UsdPhysics.FixedJoint.Define(sim.stage, "/World/DualSpatulaMount")
    joint.CreateBody0Rel().SetTargets([cfg.prim_path + "/right_arm_link7"])
    joint.CreateBody1Rel().SetTargets([case["tool"].cfg.prim_path + "/Carrier"])
    joint.CreateLocalPos0Attr(Gf.Vec3f(-0.15, 0, 0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    joint.CreateLocalRot0Attr(Gf.Quatf(0, 0, 0, 1))
    joint.CreateLocalRot1Attr(Gf.Quatf(1))
    joint.CreateExcludeFromArticulationAttr(True)
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateJointEnabledAttr(False)
    pairs = UsdPhysics.FilteredPairsAPI.Apply(sim.stage.GetPrimAtPath(case["tool"].cfg.prim_path + "/Carrier"))
    pairs.CreateFilteredPairsRel().SetTargets([cfg.prim_path + "/right_arm_link7"])
    return dict(
        robot=robot,
        mount=joint,
        ik=ik,
        base=base,
        arm_q=q,
        correction=np.zeros(3),
        rotation_correction=np.zeros(3),
    )


def drive_s1(driver, goal, rotation, actual, step, dt):
    """Drive S1 joints using measured carriage pose feedback and bounded integral correction.

    Args:
        driver: Robot, IK and control state.
        goal: Desired carriage position.
        rotation: Desired carriage rotation.
        actual: Measured carriage XYZ/XYZW pose.
        step: Physics step index.
        dt: Physics step size.
    """
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation

    robot = driver["robot"]
    stride = max(1, round(0.05 / dt))
    if step % stride == 0:
        error = goal - actual[:3]
        angle_error = (rotation * Rotation.from_quat(actual[3:]).inv()).as_rotvec()
        driver["correction"] += 2 * dt * stride * error
        driver["rotation_correction"] += 2 * dt * stride * angle_error
        driver["correction"] = np.clip(driver["correction"], -0.03, 0.03)
        driver["rotation_correction"] = np.clip(driver["rotation_correction"], -0.08, 0.08)
        corrected_rotation = Rotation.from_rotvec(driver["rotation_correction"]) * rotation
        wrist = goal + driver["correction"] - corrected_rotation.apply([0.15, 0, 0])
        wrist_rotation = (corrected_rotation * Rotation.from_euler("z", math.pi)).as_matrix()
        driver["arm_q"], _ = driver["ik"].solve(
            "right",
            wrist - driver["base"],
            0.25,
            driver["arm_q"],
            rotation=wrist_rotation,
        )
        driver["q"][0, driver["arm_ids"]] = torch.tensor(driver["arm_q"], dtype=driver["q"].dtype)
    robot.set_joint_position_target_index(target=driver["q"])
    robot.write_data_to_sim()


def prepare(case, phase, args):
    """Set the carriage goal and jaw opening for the next physical phase.

    Args:
        case: Measured case state.
        phase: Current action name.
        args: Experiment options.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    tool = case["tool"]
    case["segment_start"] = tool.data.root_link_pose_w.torch[0, :3].numpy().copy()
    case["segment_rotation"] = Rotation.from_quat(tool.data.root_link_pose_w.torch[0, 3:].numpy())
    if phase == "align":
        case["baseline"] = np.array([pose(b) for b in case["books"]])
        target = case["baseline"][1]
        case["rotation"] = Rotation.from_quat(target[3:])
        case["goal"] = np.array([-0.42, target[1], target[2]])
    elif phase == "insert":
        case["goal"][0] = -0.24
    elif phase == "close":
        case["opening"] = 0.015
    elif phase == "lift":
        case["goal"][2] += 0.04
    elif phase == "upright":
        case["rotation"] = Rotation.identity()
    elif phase == "extract":
        case["goal"][0] = -0.44
    elif phase == "shift":
        case["goal"][1] -= 0.17 if args.mode == "extract" else 0.035
    elif phase == "reinsert":
        case["goal"][0] = -0.20
    elif phase == "seat":
        case["goal"][2] = 0.972
    elif phase == "open":
        case["opening"] = 0.025
    elif phase == "withdraw":
        case["goal"][0] = -0.44
    elif phase == "return_align":
        target = pose(case["books"][3])
        case["return_baseline"] = target.copy()
        case["rotation"] = Rotation.from_quat(target[3:])
        case["goal"] = target[:3] + [-0.42, 0, 0]
    elif phase == "return_insert_blades":
        case["goal"][0] = pose(case["books"][3])[0] - 0.24
    elif phase == "return_close":
        case["opening"] = 0.015
    elif phase == "return_lift":
        case["goal"][2] += 0.08
    elif phase == "return_extract":
        case["goal"][0] = -0.44
    elif phase == "return_align_slot":
        # Center the book in the measured free interval, leaving room to open both blades.
        case["slot_bounds"] = shelf_free_slot([pose(b) for b in case["books"][:3]], case["baseline"][1, 1])
        case["slot_y"] = float(sum(case["slot_bounds"]) / 2)
        case["goal"][1] = case["slot_y"]
        case["rotation"] = Rotation.identity()
    elif phase == "return_place":
        case["goal"][0] = -0.24
    elif phase == "return_seat":
        case["goal"][2] = 0.972
    elif phase == "return_open":
        case["opening"] = 0.025
    elif phase == "return_withdraw":
        case["goal"][0] = -0.44


def summarize(case, args):
    """Score released book stability and relative motion during the lift.

    Args:
        case: Completed physical case.
        args: Experiment options.

    Returns:
        Measured result with trace and explicit success criteria.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    from isaaclab_arena_examples.library.spatula_contact_probe import tilt

    final = np.array([pose(b) for b in case["books"]])
    target = final[1]
    phases = {
        name: [r for r in case["trace"] if r["phase"] == name] for name in ("lift", "shift", "open", "withdraw", "rest")
    }
    last = phases["rest"][-1]
    distance = float(case["baseline"][1, 1] - target[1])
    target_shift = 0.17 if args.mode == "extract" else 0.035
    velocities = [b.data.root_com_vel_w.torch[0].numpy() for b in case["books"]]
    stable = all(
        p[2] > 0.90 and tilt(p[3:]) < 20 and np.linalg.norm(v[:3]) < 0.03 and np.linalg.norm(v[3:]) < 0.1
        for p, v in zip(final, velocities)
    )
    withdrawn = last["tool_pose_xyzw"][0] + 0.20 + 0.06 < min(p[0] - 0.08 for p in final)
    held = [r for r in case["trace"] if r["phase"] in ("lift", "upright", "extract", "shift", "reinsert")]
    both_contact_fraction = float(np.mean([min(r["target_normal_forces_N"]) > 0.05 for r in held]))
    relative = []
    for r in held:
        root = np.array(r["tool_pose_xyzw"])
        book = np.array(r["book_poses_xyzw"][1])
        relative.append(Rotation.from_quat(root[3:]).inv().apply(book[:3] - root[:3]))
    slip = float(np.max(np.linalg.norm(np.array(relative) - relative[0], axis=1)))
    release_forces = max(last["target_normal_forces_N"])
    success = bool(
        stable
        and tilt(target[3:]) < 15
        and abs(distance - target_shift) < 0.01
        and withdrawn
        and release_forces < 0.05
        and both_contact_fraction > 0.8
        and slip < 0.01
    )
    return dict(
        initial_angle_deg=case["angle_deg"],
        settled_tilt_deg=tilt(case["baseline"][1, 3:]),
        final_tilt_deg=tilt(target[3:]),
        lateral_shift_m=distance,
        requested_shift_m=target_shift,
        all_books_stable=bool(stable),
        blades_withdrawn=bool(withdrawn),
        final_jaw_contact_N=release_forces,
        both_jaws_contact_fraction=both_contact_fraction,
        max_book_slip_relative_to_carriage_m=slip,
        peak_jaw_net_normal_force_N=case["peak_normal_force"],
        success=success,
        baseline_book_poses_xyzw=case["baseline"].tolist(),
        final_book_poses_xyzw=final.tolist(),
        trace=case["trace"],
    )


def summarize_return(case):
    """Measure return-book lift, slip, release, seating and neighbor disturbance."""
    import numpy as np
    from scipy.spatial.transform import Rotation

    from isaaclab_arena_examples.library.spatula_contact_probe import tilt

    trace = case["trace"]
    held = [r for r in trace if r["phase"] in ("return_lift", "return_extract", "return_align_slot", "return_place")]
    relative = [
        Rotation.from_quat(r["tool_pose_xyzw"][3:])
        .inv()
        .apply(np.array(r["book_poses_xyzw"][3][:3]) - r["tool_pose_xyzw"][:3])
        for r in held
    ]
    slip = float(np.max(np.linalg.norm(np.array(relative) - relative[0], axis=1)))
    contact = float(np.mean([min(r["target_normal_forces_N"]) > 0.05 for r in held]))
    lifted = [r for r in trace if r["phase"] == "return_lift"][-1]["book_poses_xyzw"][3]
    lift = float(lifted[2] - case["return_baseline"][2])
    final = pose(case["books"][3])
    halfheight = float(np.abs(Rotation.from_quat(final[3:]).as_matrix()[2]) @ np.array([0.08, 0.0175, 0.11]))
    bottom_error = float(abs(final[2] - halfheight - 0.86))
    slot_error = float(abs(final[1] - case["slot_y"]))
    velocity = case["books"][3].data.root_com_vel_w.torch[0].numpy()
    stable = bool(tilt(final[3:]) < 5 and np.linalg.norm(velocity[:3]) < 0.01 and np.linalg.norm(velocity[3:]) < 0.05)
    last = trace[-1]
    released = bool(max(last["target_normal_forces_N"]) < 0.05)  # noqa: SIM901 - Keep JSON bool conversion.
    withdrawn = bool(last["tool_pose_xyzw"][0] + 0.26 < final[0] - 0.08)  # noqa: SIM901 - Convert NumPy bool.
    before = [r for r in trace if r["phase"] == "rest"][-1]["book_poses_xyzw"]
    disturbance = float(max(np.linalg.norm(pose(b)[:3] - before[i][:3]) for i, b in enumerate(case["books"][:3])))
    return dict(
        success=bool(
            lift > 0.04
            and slip < 0.01
            and contact > 0.8
            and bottom_error < 0.005
            and slot_error < 0.01
            and stable
            and released
            and withdrawn
            and disturbance < 0.005
        ),
        lift_m=lift,
        max_grasp_slip_m=slip,
        both_jaws_contact_fraction=contact,
        shelf_bottom_error_m=bottom_error,
        slot_center_error_m=slot_error,
        stable=stable,
        released=released,
        blades_withdrawn=withdrawn,
        max_other_book_displacement_m=disturbance,
        initial_pose_xyzw=case["return_baseline"].tolist(),
        final_pose_xyzw=final.tolist(),
        slot_y_m=case["slot_y"],
        free_slot_bounds_y_m=list(case["slot_bounds"]),
        free_slot_width_m=float(case["slot_bounds"][1] - case["slot_bounds"][0]),
    )


def run(args):
    """Run a common carriage and two physical prismatic jaws through release and withdrawal.

    Args:
        args: Parsed experiment options.
    """
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation, Slerp

    import isaaclab.sim as sim_utils
    from isaaclab_physx.physics.physx_manager_cfg import PhysxCfg

    from isaaclab_arena_examples.library.crowded_capture import GenericCapture
    from isaaclab_arena_examples.library.spatula_contact_probe import contact_diagnostics

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "probe_source.py").write_text(Path(__file__).read_text(), encoding="utf-8")
    source = args.output_dir / "source"
    source.mkdir()
    hashes = {}
    for name in (
        "dual_spatula_probe.py",
        "s1_spatula_return.py",
        "s1_library_ik.py",
        "spatula_contact_probe.py",
        "galbot_s1_robot.py",
        "crowded_capture.py",
        "render_spatula_probe.py",
        "analyze_dual_spatula.py",
    ):
        data = Path(__file__).with_name(name).read_bytes()
        (source / name).write_bytes(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    (args.output_dir / "source_manifest.json").write_text(
        json.dumps(
            dict(
                sha256=hashes,
                python=sys.version,
                executable=sys.executable,
                uid=os.geteuid(),
                options=vars(args),
            ),
            indent=2,
            default=str,
        )
        + "\n",
        encoding="utf-8",
    )
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            dt=args.dt,
            device="cpu",
            use_fabric=False,
            physics=PhysxCfg(
                enable_external_forces_every_iteration=args.external_forces_per_iteration,
                solve_articulation_contact_last=True,
            ),
        )
    )
    cases, objects = [], {}
    for index, angle in enumerate(args.angles):
        case, books = make_case(sim, args, index, angle)
        cases.append(case)
        objects.update({b.cfg.prim_path: b for b in books})
    driver = mount_s1(sim, cases[0], args) if args.s1 else None
    sim.reset()
    for case in cases:
        tool = case["tool"]
        q = tool.data.default_joint_pos.torch.clone()
        tool.write_joint_position_to_sim_index(position=q)
        tool.set_joint_position_target_index(target=q)
        case["base_id"] = torch.tensor([tool.body_names.index("Carrier")], dtype=torch.int64)
        case["jaw_body_ids"] = [
            tool.body_names.index("Left"),
            tool.body_names.index("Right"),
        ]
        case["jaw_ids"] = [
            tool.joint_names.index("jaw_left"),
            tool.joint_names.index("jaw_right"),
        ]
        tool.update(args.dt)
    if driver is not None:
        robot = driver["robot"]
        driver["q"] = robot.data.default_joint_pos.torch.clone()
        robot.write_joint_position_to_sim_index(position=driver["q"])
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(driver["q"]))
        robot.set_joint_position_target_index(target=driver["q"])
        tool = cases[0]["tool"]
        tool.write_root_pose_to_sim_index(
            root_pose=torch.tensor([*cases[0]["initial"], 0, 0, 0, 1], dtype=torch.float32).reshape(1, 7)
        )
        tool.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6)))
        sim.forward()
        robot.update(args.dt)
        tool.update(args.dt)
        driver["arm_ids"] = [robot.joint_names.index(f"right_arm_joint{i}") for i in range(1, 8)]
        driver["mount"].GetJointEnabledAttr().Set(True)
        sim.forward()
    for book in objects.values():
        book.update(args.dt)
    robots = {str(i): c["tool"] for i, c in enumerate(cases)}
    if driver is not None:
        robots["S1"] = driver["robot"]
    capture = GenericCapture(sim, robots, objects, args.output_dir, 20)
    subscription, clock, contact_pairs = contact_diagnostics(args.dt, include_robot=args.s1)
    phases = [
        ("settle", 2),
        ("align", 3),
        ("insert", 4),
        ("close", 2),
        ("lift", 2),
        ("upright", 2),
    ]
    if args.mode == "extract":
        phases += [("extract", 3), ("shift", 3), ("reinsert", 3)]
    else:
        phases += [("shift", 3)]
    phases += [("seat", 3), ("open", 2), ("withdraw", 4), ("rest", 2)]
    if args.return_book:
        phases += [
            ("return_align", 4),
            ("return_insert_blades", 4),
            ("return_close", 2),
            ("return_lift", 3),
            ("return_extract", 3),
            ("return_align_slot", 4),
            ("return_place", 4),
            ("return_seat", 3),
            ("return_open", 2),
            ("return_withdraw", 4),
            ("return_rest", 3),
        ]
    if args.probe_init:
        phases = [("settle", 0.2)]
    if args.stop_after:
        names = [name for name, _ in phases]
        assert args.stop_after in names, f"Unknown stop phase: {args.stop_after}"
        phases = phases[: names.index(args.stop_after) + 1]
    step = 0
    for phase, duration in phases:
        print("[dual-spatula] " + phase, flush=True)
        for case in cases:
            prepare(case, phase, args)
            case["slerp"] = Slerp(
                [0, 1],
                Rotation.concatenate([case["segment_rotation"], case["rotation"]]),
            )
        for k in range(round(duration / args.dt)):
            f = (k + 1) / round(duration / args.dt)
            f = f * f * (3 - 2 * f)
            for case in cases:
                tool = case["tool"]
                root = tool.data.root_link_pose_w.torch[0].numpy().copy()
                velocity = tool.data.root_com_vel_w.torch[0].numpy().copy()
                goal = case["segment_start"] * (1 - f) + case["goal"] * f
                rotation = case["slerp"](f)
                force = 900 * (goal - root[:3]) - 30 * velocity[:3]
                force[2] += 0.42 * 9.81
                force *= min(1, 30 / max(np.linalg.norm(force), 1e-12))
                torque = 8 * (rotation * Rotation.from_quat(root[3:]).inv()).as_rotvec() - 0.3 * velocity[3:]
                if args.gravity_compensation and driver is None:
                    centers = tool.data.body_com_pos_w.torch[0].numpy()
                    origin = centers[int(case["base_id"][0])]
                    for body_id in case["jaw_body_ids"]:
                        torque -= np.cross(centers[body_id] - origin, [0, 0, -0.08 * 9.81])
                torque *= min(1, 1 / max(np.linalg.norm(torque), 1e-12))
                if driver is None:
                    tool.instantaneous_wrench_composer.add_forces_and_torques_index(
                        forces=torch.tensor(force, dtype=torch.float32).reshape(1, 1, 3),
                        torques=torch.tensor(torque, dtype=torch.float32).reshape(1, 1, 3),
                        body_ids=case["base_id"],
                        is_global=True,
                    )
                else:
                    try:
                        drive_s1(driver, goal, rotation, root, step, args.dt)
                    except AssertionError as error:
                        failure = dict(success=False, phase=phase, step=step, error=str(error))
                        capture.save(failure)
                        (args.output_dir / "result.json").write_text(json.dumps(failure, indent=2) + "\n")
                        (args.output_dir / "contact_pairs.json").write_text(json.dumps(contact_pairs, indent=2) + "\n")
                        raise
                q = tool.data.joint_pos.torch.clone()
                q[0, case["jaw_ids"]] = torch.tensor([-case["opening"], case["opening"]], dtype=q.dtype)
                tool.set_joint_position_target_index(target=q)
                tool.write_data_to_sim()
            clock[0] = step
            sim.step(render=False)
            if driver is not None:
                driver["robot"].update(args.dt)
            for book in objects.values():
                book.update(args.dt)
            diagnostics = []
            for case in cases:
                case["tool"].update(args.dt)
                normal, net = [], []
                for sensor in case["sensors"]:
                    sensor.update(args.dt)
                    contact_index = 1 if phase.startswith("return_") else 0
                    normal.append(float(np.linalg.norm(sensor.data.force_matrix_w.torch[0, 0, contact_index].numpy())))
                    net.append(float(np.linalg.norm(sensor.data.net_forces_w.torch[0, 0].numpy())))
                case["peak_normal_force"] = max(case["peak_normal_force"], *net)
                record = dict(
                    time_s=(step + 1) * args.dt,
                    phase=phase,
                    tool_pose_xyzw=case["tool"].data.root_link_pose_w.torch[0].numpy().tolist(),
                    book_poses_xyzw=[pose(b).tolist() for b in case["books"]],
                    target_normal_forces_N=normal,
                    net_normal_forces_N=net,
                    jaw_positions_m=case["tool"].data.joint_pos.torch[0].tolist(),
                )
                if step % capture.stride == 0:
                    case["trace"].append(record)
                diagnostics.append(record)
            capture.record(step, phase, {"cases": diagnostics})
            step += 1
    subscription.unsubscribe()
    (args.output_dir / "contact_pairs.json").write_text(json.dumps(contact_pairs, indent=2) + "\n", encoding="utf-8")
    if args.probe_init:
        print(json.dumps(contact_pairs, indent=2), flush=True)
        return
    results = (
        [dict(initial_angle_deg=c["angle_deg"], success=False, trace=c["trace"]) for c in cases]
        if args.stop_after
        else [summarize(case, args) for case in cases]
    )
    if args.return_book and not args.stop_after:
        for case, result in zip(cases, results):
            result["return"] = summarize_return(case)
            result["success"] = result["success"] and result["return"]["success"]
    report = dict(
        scope=(
            "S1 wrist-mounted two-jaw gripper; gravity enabled; check s1_self_collisions for collision scope"
            if args.s1
            else "Floating common carriage with two physical prismatic joints; not S1 arm validation"
        ),
        drive_mode=("S1 joint drives with TCP feedback" if args.s1 else "force-driven carriage"),
        physics="PhysX",
        s1_self_collisions=args.s1_self_collisions if args.s1 else None,
        uid=os.geteuid(),
        dt=args.dt,
        external_forces_every_iteration=args.external_forces_per_iteration,
        solve_articulation_contact_last=True,
        layout=args.layout,
        mode=args.mode,
        return_book=args.return_book,
        stop_after=args.stop_after,
        case_spacing_m=0.8,
        thickness_m=0.003,
        jaw_force_limit_N=args.jaw_force,
        gravity_compensation=bool(args.gravity_compensation and not args.s1),
        friction=args.friction,
        carriage_force_limit_N=None if args.s1 else 30,
        carriage_torque_limit_Nm=None if args.s1 else 1,
        tool_mass_kg=0.42,
        book_mass_kg=0.18,
        cases=results,
        success=all(r["success"] for r in results),
        limitations=[
            "Rigid books and blades; damage and bending are not modeled.",
            (
                "S1 motor dynamics and friction are uncalibrated."
                if args.s1
                else "The carriage is force-driven, not mounted to S1 in this component test."
            ),
            "Motor force caps do not bound contact impulses.",
            "Shelf-front manipulation only; no mobile-base travel or return-box pickup in this contact study.",
        ],
    )
    capture.save(report)
    (args.output_dir / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (args.output_dir / "contact_pairs.json").write_text(json.dumps(contact_pairs, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            [
                {
                    k: v
                    for k, v in r.items()
                    if k
                    not in (
                        "trace",
                        "baseline_book_poses_xyzw",
                        "final_book_poses_xyzw",
                    )
                }
                for r in results
            ],
            indent=2,
        ),
        flush=True,
    )


def main():
    """Launch the existing Isaac Sim runtime for the physical gripper test."""
    args = parse_args()
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

    with SimulationAppContext(args):
        run(args)


if __name__ == "__main__":
    main()
