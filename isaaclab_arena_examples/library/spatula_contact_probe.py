# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Measure thin-tool contact feasibility with freely moving upright and leaning books."""

import argparse
import json
import math
import os
import re
from pathlib import Path

WRIST_TOOL_OFFSET = 0.35


def parse_args():
    """Return contact-probe options and Isaac Sim launch options."""
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--angles", type=float, nargs="+", default=[0, 10, -10, 20, -20])
    parser.add_argument("--thickness", type=float, default=0.003)
    parser.add_argument("--dt", type=float, default=1 / 240)
    parser.add_argument("--force-limit", type=float, default=10.0)
    parser.add_argument(
        "--push-height",
        type=float,
        default=0.11,
        help="Tool center height above the book's local bottom.",
    )
    parser.add_argument("--contact-offset", type=float, default=0.0005)
    parser.add_argument(
        "--diagnose-only",
        action="store_true",
        help="Record the first 0.1 s of contact pairs only.",
    )
    parser.add_argument(
        "--gap",
        type=float,
        default=0.006,
        help="Initial horizontal book-envelope clearance in metres.",
    )
    parser.add_argument(
        "--s1",
        action="store_true",
        help="Drive the tool through S1 joint targets instead of external forces.",
    )
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    assert args.device == "cpu", "Use --device cpu for reproducible small contact probes."
    assert args.thickness > 0 and args.dt > 0 and args.force_limit > 0
    assert args.gap > 0
    assert 0.02 < args.push_height < 0.20
    assert not args.s1 or len(args.angles) == 1, "S1 mode supports one shelf case per run."
    assert 0 < args.contact_offset < args.thickness / 2
    assert not args.output_dir.exists() or not any(args.output_dir.iterdir()), "Use a fresh output directory."
    args.headless, args.visualizer = True, []
    return args


def spawn_tool_robot(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Spawn the official S1 with its right stock gripper removed."""
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from pxr import Usd

    root = spawn_from_usd(
        prim_path,
        cfg,
        translation=translation,
        orientation=orientation,
        **kwargs,
    )
    for prim in list(Usd.PrimRange(root)):
        if not prim.IsValid():
            continue
        name = prim.GetName()
        if name.startswith("right_") and any(
            part in name
            for part in (
                "active_link",
                "active_joint",
                "passive_link",
                "passive_joint",
                "gripper_base_link",
            )
        ):
            prim.SetActive(False)
    return root


def contact_diagnostics(dt, include_robot=False):
    """Return a contact subscription and per-collider impulse and penetration summaries.

    Args:
        dt: Physics step in seconds.
        include_robot: Also record S1 contacts for mounting and self-collision diagnosis.

    Returns:
        Subscription, mutable physics clock, and collider-pair measurements.
    """
    import numpy as np

    import omni.physx
    from pxr import PhysicsSchemaTools

    clock, pairs = [0], {}

    def record(headers, data):
        for header in headers:
            actors = [str(PhysicsSchemaTools.intToSdfPath(x)) for x in (header.actor0, header.actor1)]
            if not any("/Tool" in actor or (include_robot and "/Galbot/" in actor) for actor in actors):
                continue
            colliders = [str(PhysicsSchemaTools.intToSdfPath(x)) for x in (header.collider0, header.collider1)]
            key = " | ".join(colliders)
            entry = pairs.setdefault(
                key,
                dict(
                    actors=actors,
                    colliders=colliders,
                    peak_impulse_Ns=0,
                    max_penetration_m=0,
                    contacts=0,
                ),
            )
            for index in range(
                header.contact_data_offset,
                header.contact_data_offset + header.num_contact_data,
            ):
                contact = data[index]
                impulse = float(np.linalg.norm(list(contact.impulse)))
                entry["contacts"] += 1
                entry["max_penetration_m"] = max(entry["max_penetration_m"], -float(contact.separation))
                if impulse > entry["peak_impulse_Ns"]:
                    entry.update(
                        peak_impulse_Ns=impulse,
                        peak_time_s=clock[0] * dt,
                        peak_position_m=list(contact.position),
                        peak_separation_m=float(contact.separation),
                    )

    subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(record)
    return subscription, clock, pairs


def state(body):
    """Return measured XYZ/XYZW pose and COM velocity for a rigid body."""
    return (
        body.data.root_link_pose_w.torch[0].numpy().copy(),
        body.data.root_com_vel_w.torch[0].numpy().copy(),
    )


def tilt(q):
    """Return the unsigned inclination of a measured XYZW quaternion in degrees."""
    import numpy as np
    from scipy.spatial.transform import Rotation

    return float(np.degrees(np.arccos(np.clip(Rotation.from_quat(q).apply([0, 0, 1])[2], -1, 1))))


def run(args):
    """Run a force-limited dynamic tool without grasp constraints or book pose writes.

    Args:
        args: Parsed launch and contact-probe options.
    """
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation

    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from isaaclab.sensors import ContactSensor, ContactSensorCfg

    from isaaclab_arena_examples.library.crowded_capture import GenericCapture

    assert os.geteuid() != 0, "Run physical probes as a non-root user."
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "probe_source.py").write_text(Path(__file__).read_text(), encoding="utf-8")
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=args.dt, device="cpu", use_fabric=False))
    material = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4, restitution=0)
    book_size = np.array([0.16, 0.035, 0.22])
    tool_size = np.array([0.12, args.thickness, 0.04])
    objects, cases = {}, []

    def box(path, size, position, color, mass=None, angle=0):
        cfg = sim_utils.CuboidCfg(
            size=tuple(size),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=args.contact_offset, rest_offset=0),
            physics_material=material,
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
        rotation = (math.sin(angle / 2), 0, 0, math.cos(angle / 2))
        if mass is None:
            cfg.func(path, cfg, translation=tuple(position), orientation=rotation)
            return None
        body = RigidObject(
            RigidObjectCfg(
                prim_path=path,
                spawn=cfg,
                init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(position), rot=rotation),
            )
        )
        objects[path] = body
        return body

    for index, angle_deg in enumerate(args.angles):
        cy = index * 0.65
        root = f"/World/Case{index}"
        box(root + "/Shelf", (0.55, 0.48, 0.04), (0, cy, 0.84), (0.4, 0.27, 0.16))
        # Ends arrest a falling row, but every book in the row is dynamic.
        for sign in (-1, 1):
            box(
                root + f"/End{sign+1}",
                (0.20, 0.012, 0.26),
                (0, cy + sign * 0.17, 0.99),
                (0.3, 0.33, 0.35),
            )
        books = []
        half_width = (
            0.035 * abs(math.cos(math.radians(angle_deg))) + 0.22 * abs(math.sin(math.radians(angle_deg)))
        ) / 2
        pitch = half_width + 0.0175 + args.gap
        for j in (-1, 0, 1):
            theta = math.radians(angle_deg) if j == 0 else 0
            z = 0.86 + (0.22 * abs(math.cos(theta)) + 0.035 * abs(math.sin(theta))) / 2 + 0.001
            books.append(
                box(
                    root + f"/Book{j+1}",
                    book_size,
                    (0, cy + j * pitch, z),
                    (0.65, 0.12, 0.09) if j == 0 else (0.15, 0.4, 0.28),
                    0.18,
                    theta,
                )
            )
        tool_start = np.array([-0.22, cy + 0.025, 0.98])
        tool = box(root + "/Tool", tool_size, tool_start, (0.55, 0.64, 0.72), 0.12)
        sensor = ContactSensor(
            ContactSensorCfg(
                prim_path=tool.cfg.prim_path,
                update_period=0,
                filter_prim_paths_expr=[b.cfg.prim_path for b in books],
            )
        )
        cases.append(
            dict(
                angle_deg=angle_deg,
                cy=cy,
                books=books,
                tool=tool,
                start=tool_start,
                goal=tool_start.copy(),
                rotation=Rotation.identity(),
                trace=[],
                baseline=None,
                insertion=None,
                sensor=sensor,
                peak_normal_force=0,
            )
        )
    robot = None
    if args.s1:
        from isaaclab.actuators import ImplicitActuatorCfg
        from isaaclab.assets import Articulation
        from pxr import Gf, UsdPhysics

        from isaaclab_arena_examples.library.galbot_s1_robot import make_galbot_cfg
        from isaaclab_arena_examples.library.s1_library_ik import S1Kinematics

        base = np.array([-1.08, 0.20, 0])
        kinematics = S1Kinematics()
        cfg = make_galbot_cfg(position=tuple(base))
        cfg.spawn.func = spawn_tool_robot
        cfg.spawn.rigid_props.disable_gravity = False
        cfg.actuators.pop("hands")
        cfg.actuators["stock_left_gripper"] = ImplicitActuatorCfg(
            joint_names_expr=["left_(active|passive)_joint.*"], stiffness=20, damping=1
        )
        # Use official URDF effort and velocity limits, not unlimited joint drives.
        for group in cfg.actuators.values():
            matching = {
                n: j
                for n, j in kinematics.by_name.items()
                if j.find("limit") is not None and any(re.fullmatch(pattern, n) for pattern in group.joint_names_expr)
            }
            group.effort_limit_sim = {n: float(j.find("limit").get("effort")) for n, j in matching.items()}
            group.velocity_limit_sim = {n: float(j.find("limit").get("velocity")) for n, j in matching.items()}
        seed = [cfg.init_state.joint_pos[f"right_arm_joint{i}"] for i in range(1, 8)]
        arm_q, _ = kinematics.solve("right", cases[0]["start"] - [WRIST_TOOL_OFFSET, 0, 0] - base, 0.25, seed)
        cfg.init_state.joint_pos.update({f"right_arm_joint{i}": float(v) for i, v in enumerate(arm_q, 1)})
        robot = Articulation(cfg)
        # The existing S1 hand mount starts 140 mm beyond link7's frame. Starting
        # a shank at that frame instead intersects the link5/link6 collision meshes.
        shank_cfg = sim_utils.CuboidCfg(
            size=(0.15, 0.020, 0.025),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=args.contact_offset, rest_offset=0),
            physics_material=material,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.4, 0.45, 0.5)),
        )
        shank_cfg.func(
            cases[0]["tool"].cfg.prim_path + "/Shank",
            shank_cfg,
            translation=(-0.135, 0, 0),
        )
        joint = UsdPhysics.FixedJoint.Define(sim.stage, "/World/SpatulaMount")
        joint.CreateBody0Rel().SetTargets([cfg.prim_path + "/right_arm_link7"])
        joint.CreateBody1Rel().SetTargets([cases[0]["tool"].cfg.prim_path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(-WRIST_TOOL_OFFSET, 0, 0))
        joint.CreateLocalRot0Attr(Gf.Quatf(0, 0, 0, 1))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1))
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateCollisionEnabledAttr(False)
        joint.CreateJointEnabledAttr(False)
        # The shank overlaps its own wrist mounting flange. Exclude only this adjacent
        # connected pair; tool/book, tool/shelf and other robot links still collide.
        mount_pairs = UsdPhysics.FilteredPairsAPI.Apply(sim.stage.GetPrimAtPath(cases[0]["tool"].cfg.prim_path))
        mount_pairs.CreateFilteredPairsRel().SetTargets([cfg.prim_path + "/right_arm_link7"])
    sim.reset()
    if robot is not None:
        q = robot.data.default_joint_pos.torch.clone()
        robot.write_joint_position_to_sim_index(position=q)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q))
        robot.set_joint_position_target_index(target=q)
        tool_pose = torch.tensor([*cases[0]["start"], 0, 0, 0, 1], dtype=torch.float32).reshape(1, 7)
        cases[0]["tool"].write_root_pose_to_sim_index(root_pose=tool_pose)
        cases[0]["tool"].write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6)))
        sim.forward()
        robot.update(args.dt)
        wrist_id = robot.body_names.index("right_arm_link7")
        measured_wrist = robot.data.body_link_pose_w.torch[0, wrist_id].numpy().copy()
        expected_wrist = cases[0]["start"] - [WRIST_TOOL_OFFSET, 0, 0]
        print(
            f"[spatula] initial wrist measured={measured_wrist.tolist()} expected={expected_wrist.tolist()}",
            flush=True,
        )
        assert (
            np.linalg.norm(measured_wrist[:3] - expected_wrist) < 0.01
        ), "S1 initial FK disagrees with measured wrist."
        joint.GetJointEnabledAttr().Set(True)
        sim.forward()
        arm_ids = [robot.joint_names.index(f"right_arm_joint{i}") for i in range(1, 8)]
    for obj in objects.values():
        obj.update(args.dt)
    capture = GenericCapture(sim, robot if robot is not None else {}, objects, args.output_dir, 20)
    subscription, physics_clock, contact_pairs = contact_diagnostics(args.dt)
    phases = [
        ("settle", 2),
        ("align", 2),
        ("insert", 4),
        ("push", 3),
        ("withdraw", 4),
        ("rest", 2),
    ]
    if args.diagnose_only:
        phases = [("settle", 0.1)]
    step = 0

    for phase, duration in phases:
        print(f"[spatula] {phase}", flush=True)
        for case in cases:
            case["segment_start"] = state(case["tool"])[0][:3]
            if phase == "align":
                poses = [state(b)[0] for b in case["books"]]
                case["baseline"] = np.array(poses)
                pose = poses[1]
                rotation = Rotation.from_quat(pose[3:])
                case["rotation"] = rotation
                # Approach along shelf depth, parallel to the measured book face.
                case["normal"] = rotation.apply([0, 1, 0])
                # Aim at the slot midpoint. Centering 3 mm from the blade's face
                # puts its opposite edge on a neighbor in a 6 mm gap.
                case["entry"] = pose[:3] + rotation.apply([0, 0.0175 + args.gap / 2, args.push_height - 0.11])
                case["entry"][0] = -0.22
                case["goal"] = case["entry"].copy()
            elif phase == "insert":
                case["goal"] = case["entry"].copy()
                case["goal"][0] = -0.04
            elif phase == "push":
                case["insertion"] = state(case["tool"])[0].copy()
                case["goal"] = case["goal"] - case["normal"] * 0.04
            elif phase == "withdraw":
                case["goal"] = case["goal"].copy()
                case["goal"][0] = -0.22
        count = round(duration / args.dt)
        for local_step in range(count):
            f = (local_step + 1) / count
            f = f * f * (3 - 2 * f)
            diagnostics = []
            for case in cases:
                pose, velocity = state(case["tool"])
                target = case["segment_start"] * (1 - f) + case["goal"] * f
                force = 500 * (target - pose[:3]) - 12 * velocity[:3]
                # The force limit includes gravity compensation; no kinematic tool writes.
                force[2] += 0.12 * 9.81
                force *= min(1, args.force_limit / max(np.linalg.norm(force), 1e-12))
                error = (case["rotation"] * Rotation.from_quat(pose[3:]).inv()).as_rotvec()
                torque = 0.6 * error - 0.025 * velocity[3:]
                torque *= min(1, 0.2 / max(np.linalg.norm(torque), 1e-12))
                if robot is None:
                    case["tool"].instantaneous_wrench_composer.add_forces_and_torques_index(
                        forces=torch.tensor(force, dtype=torch.float32).reshape(1, 1, 3),
                        torques=torch.tensor(torque, dtype=torch.float32).reshape(1, 1, 3),
                        is_global=True,
                    )
                    case["tool"].write_data_to_sim()
                else:
                    # Piecewise joint targets are solved at 20 Hz; PhysX integrates every dt.
                    if step % max(1, round(0.05 / args.dt)) == 0:
                        wrist_rotation = case["rotation"].as_matrix() @ Rotation.from_euler("z", math.pi).as_matrix()
                        wrist = target - case["rotation"].apply([WRIST_TOOL_OFFSET, 0, 0])
                        arm_q, _ = kinematics.solve("right", wrist - base, 0.25, arm_q, rotation=wrist_rotation)
                        q[0, arm_ids] = torch.tensor(arm_q, dtype=q.dtype)
                    robot.set_joint_position_target_index(target=q)
                    robot.write_data_to_sim()
                diagnostics.append(
                    dict(
                        command_force_N=force.tolist(),
                        command_torque_Nm=torque.tolist(),
                        tracking_error_m=float(np.linalg.norm(target - pose[:3])),
                    )
                )
            physics_clock[0] = step
            sim.step(render=False)
            if robot is not None:
                robot.update(args.dt)
            for obj in objects.values():
                obj.update(args.dt)
            for case, diagnostic in zip(cases, diagnostics):
                case["sensor"].update(args.dt)
                reaction = case["sensor"].data.net_forces_w.torch[0, 0].numpy().copy()
                diagnostic["net_normal_contact_force_N"] = reaction.tolist()
                case["peak_normal_force"] = max(case["peak_normal_force"], float(np.linalg.norm(reaction)))
                if args.s1:
                    diagnostic["standalone_reference_force_N"] = diagnostic.pop("command_force_N")
            if step % capture.stride == 0:
                for case, diagnostic in zip(cases, diagnostics):
                    case["trace"].append(
                        dict(
                            time_s=step * args.dt,
                            phase=phase,
                            tool_pose_xyzw=state(case["tool"])[0].tolist(),
                            book_poses_xyzw=[state(b)[0].tolist() for b in case["books"]],
                            book_velocities=[state(b)[1].tolist() for b in case["books"]],
                            **diagnostic,
                        )
                    )
            capture.record(step, phase, {"cases": diagnostics})
            step += 1
    (args.output_dir / "contact_pairs.json").write_text(json.dumps(contact_pairs, indent=2) + "\n", encoding="utf-8")
    subscription.unsubscribe()
    if args.diagnose_only:
        print(json.dumps(contact_pairs, indent=2), flush=True)
        return
    results = []
    for case in cases:
        baseline = case["baseline"]
        final = np.array([state(b)[0] for b in case["books"]])

        push_trace = [r for r in case["trace"] if r["phase"] == "push"]
        displacement = float(np.dot(final[1, :3] - baseline[1, :3], -case["normal"]))
        insertion = float(case["insertion"][0] + tool_size[0] / 2 + book_size[0] / 2)
        stable = all(
            tilt(p[3:]) < 45 and p[2] > 0.90 and np.linalg.norm(state(b)[1][:3]) < 0.03
            for p, b in zip(final, case["books"])
        )
        target_half_y = np.abs(Rotation.from_quat(final[1, 3:]).as_matrix()[1]) @ (book_size / 2)
        neighbor_half_y = np.abs(Rotation.from_quat(final[2, 3:]).as_matrix()[1]) @ (book_size / 2)
        clear_width = float(final[2, 1] - neighbor_half_y - final[1, 1] - target_half_y)
        result = dict(
            initial_angle_deg=case["angle_deg"],
            settled_tilt_deg=tilt(baseline[1, 3:]),
            final_tilt_deg=tilt(final[1, 3:]),
            insertion_depth_m=insertion,
            target_displacement_along_push_m=displacement,
            all_books_stable=bool(stable),
            translation_probe_passed=bool(insertion >= 0.06 and displacement >= 0.03 and stable),
            final_vertical_corridor_width_m=clear_width,
            success=bool(insertion >= 0.06 and clear_width >= 0.041 and stable and tilt(final[1, 3:]) <= 15),
            max_command_force_N=(None if args.s1 else max(np.linalg.norm(r["command_force_N"]) for r in case["trace"])),
            peak_net_normal_contact_force_N=case["peak_normal_force"],
            max_push_tracking_error_m=max(r["tracking_error_m"] for r in push_trace),
            baseline_book_poses_xyzw=baseline.tolist(),
            final_book_poses_xyzw=final.tolist(),
            trace=case["trace"],
        )
        results.append(result)
    report = dict(
        scope=(
            "S1 joint-driven mounted tool contact; fixed chassis, gravity enabled, self collision disabled"
            if args.s1
            else "Dynamic standalone tool contact feasibility; not an S1 arm or grasp validation"
        ),
        physics="PhysX",
        uid=os.geteuid(),
        dt=args.dt,
        thickness_m=args.thickness,
        force_limit_N=args.force_limit,
        torque_limit_Nm=0.2,
        contact_offset_m=args.contact_offset,
        friction=dict(static=0.5, dynamic=0.4),
        initial_envelope_gap_m=args.gap,
        push_height_above_book_bottom_m=args.push_height,
        book_mass_kg=0.18,
        tool_mass_kg=0.12,
        cases=results,
        contact_forces_measured=True,
        limitations=[
            "Contact sensor reports net normal force; opposing contacts can cancel and tangential friction is omitted.",
            "Rigid tools do not model bending or paper damage.",
            "Book initial angles are released before action; use measured settled angles.",
            "No hand grasp, extraction, paper damage, or tool bending is tested here.",
        ],
    )
    report["drive_mode"] = "S1 joint position drives" if args.s1 else "force-limited standalone dynamic tool"
    if args.s1:
        report["force_limit_N"] = None
        report["torque_limit_Nm"] = None
        report["limitations"].append(
            "S1 contact force is not limited by the standalone tool force cap; commanded wrench is a diagnostic only."
        )
    capture.save(report)
    (args.output_dir / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
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
    """Launch Isaac Sim and run the contact experiment."""
    args = parse_args()
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

    with SimulationAppContext(args):
        run(args)


if __name__ == "__main__":
    main()
