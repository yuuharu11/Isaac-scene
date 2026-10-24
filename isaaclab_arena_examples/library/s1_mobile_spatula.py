# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Drive the original library's S1 through real steering and wheel contact."""

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

from isaaclab_arena_examples.library.s1_library_plan import full_plan
from isaaclab_arena_examples.library.s1_mobile_control import (
    WheelController,
    control_tools,
    grounded_base_height,
    jaw_motion_settled,
    measure_round_wheels,
    prepare_full_phase,
    record_jaw_settling,
    robot_config,
    set_grasp_force_limit,
    validate_release_support,
    validate_transport_clearance,
)


def parse_args():
    """Parse physical full-scene and representative-probe options."""
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--probe-drive", action="store_true")
    parser.add_argument("--batch-books", type=int, choices=(1, 2), default=1)
    parser.add_argument("--expanded-room", action="store_true")
    parser.add_argument("--dense-shelves", action="store_true")
    parser.add_argument(
        "--packed-shelves", action="store_true", help="Uniform task-row pitch; clear space through contact."
    )
    parser.add_argument("--motion-speed", type=float, default=1, help="Arm trajectory speed multiplier (1–2).")
    parser.add_argument(
        "--wheel-speed-limit",
        type=float,
        default=1.5,
        help="Simulated wheel rad/s limit; above 1.5 is an uncalibrated faster-drive scenario.",
    )
    parser.add_argument("--travel-speed", type=float, default=0.09)
    parser.add_argument("--travel-acceleration", type=float, default=0.06)
    parser.add_argument("--arrival-gain", type=float, default=0.8, help="Position feedback gain for wheel navigation.")
    parser.add_argument("--jaw-force", type=float, default=4, help="Per-jaw actuator force limit in newtons.")
    parser.add_argument("--loan-jaw-force", type=float, help="Optional lower per-jaw limit for borrowed books.")
    parser.add_argument("--max-jaw-wait", type=float, default=6, help="Bounded adaptive jaw wait in seconds.")
    parser.add_argument("--cylinder-wheels", action="store_true", help="Use native round cylinder contact for wheels.")
    parser.add_argument(
        "--probe-shelf", action="store_true", help="Initialize at shelf A for a blocker-contact diagnostic."
    )
    parser.add_argument("--stop-after")
    parser.add_argument("--diagnostic-phase", help="Save measured state at 240 Hz for one phase.")
    parser.add_argument(
        "--jaw-close-duration",
        type=float,
        default=0,
        help="Seconds for a smooth jaw motor target ramp on all books.",
    )
    parser.add_argument(
        "--blocker-close-duration",
        type=float,
        default=0,
        help="Seconds for a smooth tilted-book jaw motor target ramp; zero is legacy closing.",
    )
    parser.add_argument(
        "--adaptive-grasp-wait",
        action="store_true",
        help="End jaw waits only after sustained measured contact or release and book stability.",
    )
    parser.add_argument(
        "--probe-high", action="store_true", help="Initialize at shelf B for high-shelf contact diagnostics."
    )
    parser.add_argument(
        "--arm-control-period",
        type=float,
        default=0.05,
        help="Arm IK feedback period in seconds; legacy value is 0.05.",
    )
    parser.add_argument("--dt", type=float, default=1 / 240)
    parser.add_argument("--blocker-angle", type=float, default=-10)
    parser.set_defaults(friction=0.5)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.packed_shelves:
        args.dense_shelves = True
        assert abs(args.blocker_angle) < 1e-9, "Packed shelves require --blocker-angle 0."
    assert 1.5 <= args.wheel_speed_limit <= 6
    assert 0 < args.travel_speed <= args.wheel_speed_limit * 0.08272846
    assert 0 < args.travel_acceleration <= 0.25
    assert 0.8 <= args.arrival_gain <= 1.5
    assert 1 <= args.motion_speed <= 2
    assert not args.dense_shelves or args.batch_books == 1
    assert 0 < args.jaw_force <= 4
    assert args.loan_jaw_force is None or 0 < args.loan_jaw_force <= args.jaw_force
    assert 2 <= args.max_jaw_wait <= 10
    assert 0 <= args.jaw_close_duration <= 1.25
    assert 0 <= args.blocker_close_duration <= 1.25
    assert args.dt <= args.arm_control_period <= 0.05
    assert os.geteuid() != 0 and args.device == "cpu"
    assert sum((args.probe_drive, args.probe_shelf, args.probe_high)) <= 1
    assert not args.output_dir.exists() or not any(args.output_dir.iterdir())
    args.headless, args.visualizer = True, []
    return args


def drive_probe(args):
    """Verify wheel-floor traction in the original full-size library before book transport."""
    import numpy as np

    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab_physx.physics.physx_manager_cfg import PhysxCfg

    from isaaclab_arena_examples.library.crowded_capture import GenericCapture
    from isaaclab_arena_examples.library.s1_library_scene import create_library
    from isaaclab_arena_examples.library.spatula_contact_probe import contact_diagnostics

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            dt=args.dt,
            device="cpu",
            use_fabric=False,
            enable_scene_query_support=args.cylinder_wheels,
            physics=PhysxCfg(enable_external_forces_every_iteration=True, solve_articulation_contact_last=True),
        )
    )
    books, _ = create_library(sim)
    robot = Articulation(robot_config((-0.18, -2.31, 0), args.cylinder_wheels, args.wheel_speed_limit))
    sim.reset()
    q = robot.data.default_joint_pos.torch.clone()
    robot.write_joint_position_to_sim_index(position=q)
    robot.set_joint_position_target_index(target=q)
    sim.forward()
    robot.update(args.dt)
    wheels = WheelController(
        robot, args.travel_speed, args.travel_acceleration, args.wheel_speed_limit, args.arrival_gain
    )
    capture = GenericCapture(sim, robot, books, args.output_dir, 10)
    subscription, clock, contacts = contact_diagnostics(args.dt, include_robot=True)
    step = 0
    trajectory = []
    for phase, duration, goal in [
        ("settle", 3, [-0.18, -2.31]),
        ("drive_probe", 12, [-0.18, -2.01]),
        ("done", 4, [-0.18, -2.01]),
    ]:
        print("[mobile] " + phase, flush=True)
        for _ in range(round(duration / args.dt)):
            wheels.command(goal, q, args.dt)
            clock[0] = step
            sim.step(render=False)
            robot.update(args.dt)
            for book in books.values():
                book.update(args.dt)
            capture.record(step, phase)
            if step % 240 == 0:
                trajectory.append(
                    dict(
                        time=step * args.dt,
                        pose=robot.data.root_link_pose_w.torch[0].tolist(),
                        vel=robot.data.root_com_vel_w.torch[0].tolist(),
                        wheel_q=robot.data.joint_pos.torch[0, wheels.steer].tolist(),
                        wheel_v=robot.data.joint_vel.torch[0, wheels.drive].tolist(),
                    )
                )
            step += 1
    subscription.unsubscribe()
    final = robot.data.root_link_pose_w.torch[0].numpy()
    result = dict(
        success=bool(np.linalg.norm(final[:2] - [-0.18, -2.01]) < 0.015),  # noqa: SIM901 - Convert NumPy bool.
        final_pose=final.tolist(),
        trajectory=trajectory,
        base_drive="steering and wheel velocity motors; floor contact; no root writes after initialization",
        uid=os.geteuid(),
    )
    capture.save(result)
    (args.output_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    (args.output_dir / "contact_pairs.json").write_text(json.dumps(contacts, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)


def create_full_scene(args):
    """Create the original room, floating S1, dynamic neighbors and two wrist-mounted tools."""
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation

    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab_physx.physics.physx_manager_cfg import PhysxCfg
    from pxr import Gf, UsdPhysics

    from isaaclab_arena_examples.library.dual_spatula_probe import make_dual_tool
    from isaaclab_arena_examples.library.s1_library_ik import S1Kinematics
    from isaaclab_arena_examples.library.s1_library_scene import create_library

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            dt=args.dt,
            device="cpu",
            use_fabric=False,
            enable_scene_query_support=args.cylinder_wheels,
            physics=PhysxCfg(enable_external_forces_every_iteration=True, solve_articulation_contact_last=True),
        )
    )
    books, metadata = create_library(
        sim,
        dynamic_neighbors=True,
        blocker_angle=args.blocker_angle,
        batch_books=args.batch_books,
        expanded_room=args.expanded_room,
        dense_shelves=args.dense_shelves,
        packed_shelves=args.packed_shelves,
    )
    base = np.array([0.30, 0, 0]) if args.probe_shelf else np.array([-0.18, -2.31, 0])
    if args.probe_high:
        base = np.array([0.30, 1.80, 0])
    ik = S1Kinematics()
    base[2] = grounded_base_height(ik, metadata["floor_z"]) if args.cylinder_wheels else base[2]
    initial_lift = 0.65 if args.probe_high else 0.25
    cfg = robot_config(base, args.cylinder_wheels, args.wheel_speed_limit)
    cfg.init_state.joint_pos["torso_lift_joint1"] = initial_lift
    import re

    for name, actuator in cfg.actuators.items():
        if name in ("steering", "driving"):
            continue
        matching = {
            n: j
            for n, j in ik.by_name.items()
            if j.find("limit") is not None and any(re.fullmatch(p, n) for p in actuator.joint_names_expr)
        }
        actuator.effort_limit_sim = {n: float(j.find("limit").get("effort")) for n, j in matching.items()}
        actuator.velocity_limit_sim = {n: float(j.find("limit").get("velocity")) for n, j in matching.items()}
    starts = {"left": np.array([0.41, -2.10, 0.97]), "right": np.array([0.44, -2.56, 1.10])}
    if args.probe_shelf:
        starts = {"left": np.array([0.92, 0.25, 1.25]), "right": np.array([0.92, -0.25, 1.10])}
    if args.probe_high:
        starts = {"left": np.array([0.86, 2.05, 1.50]), "right": np.array([0.86, 1.55, 1.50])}
    for side, start in starts.items():
        seed = [cfg.init_state.joint_pos[f"{side}_arm_joint{i}"] for i in range(1, 8)]
        q, _ = ik.solve(side, start - [0.15, 0, 0] - base, initial_lift, seed)
        cfg.init_state.joint_pos.update({f"{side}_arm_joint{i}": float(v) for i, v in enumerate(q, 1)})
    robot = Articulation(cfg)
    target_names = list(metadata["catalog"])
    target_paths = [books[name].cfg.prim_path for name in target_names]
    tools = {}
    for side, start in starts.items():
        root = "/World/Tool" + side.capitalize()
        tool, sensors = make_dual_tool(sim, args, root, start, target_paths)
        joint = UsdPhysics.FixedJoint.Define(sim.stage, "/World/Mount" + side.capitalize())
        wrist = cfg.prim_path + f"/{side}_arm_link7"
        joint.CreateBody0Rel().SetTargets([wrist])
        joint.CreateBody1Rel().SetTargets([root + "/Carrier"])
        joint.CreateLocalPos0Attr(Gf.Vec3f(-0.15, 0, 0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0))
        joint.CreateLocalRot0Attr(Gf.Quatf(0, 0, 0, 1))
        joint.CreateLocalRot1Attr(Gf.Quatf(1))
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateCollisionEnabledAttr(False)
        joint.CreateJointEnabledAttr(False)
        UsdPhysics.FilteredPairsAPI.Apply(
            sim.stage.GetPrimAtPath(root + "/Carrier")
        ).CreateFilteredPairsRel().SetTargets([wrist])
        tools[side] = dict(
            tool=tool,
            sensors=sensors,
            mount=joint,
            goal=start.copy(),
            rotation=Rotation.identity(),
            opening=0.025,
            held=None,
            active=None,
            correction=np.zeros(3),
            angular_correction=np.zeros(3),
        )
    sim.reset()
    q = robot.data.default_joint_pos.torch.clone()
    robot.write_joint_position_to_sim_index(position=q)
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q))
    robot.set_joint_position_target_index(target=q)
    for side, t in tools.items():
        tool = t["tool"]
        tq = tool.data.default_joint_pos.torch.clone()
        tool.write_joint_position_to_sim_index(position=tq)
        tool.set_joint_position_target_index(target=tq)
        tool.write_root_pose_to_sim_index(
            root_pose=torch.tensor([*starts[side], 0, 0, 0, 1], dtype=torch.float32).reshape(1, 7)
        )
        tool.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6)))
    sim.forward()
    robot.update(args.dt)
    for t in tools.values():
        t["tool"].update(args.dt)
        t["mount"].GetJointEnabledAttr().Set(True)
    sim.forward()
    for book in books.values():
        book.update(args.dt)
    return sim, robot, books, tools, q, ik, metadata


def snapshot_sources(args):
    """Save runtime source hashes and native execution options beside the measured capture."""
    source = args.output_dir / "source"
    source.mkdir(parents=True, exist_ok=True)
    hashes = {}
    for name in (
        "s1_mobile_spatula.py",
        "dual_spatula_probe.py",
        "s1_library_scene.py",
        "s1_library_room.py",
        "s1_library_plan.py",
        "s1_mobile_control.py",
        "s1_library_diagnostics.py",
        "asset_materials.py",
        "material_renderer.py",
        "s1_library_ik.py",
        "galbot_s1_robot.py",
        "spatula_contact_probe.py",
        "crowded_capture.py",
        "render_s1_library.py",
        "render_video.py",
        "analyze_mobile_spatula.py",
    ):
        data = Path(__file__).with_name(name).read_bytes()
        (source / name).write_bytes(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    metadata = dict(
        sha256=hashes,
        uid=os.geteuid(),
        python=sys.version,
        executable=sys.executable,
        options=vars(args),
        args=sys.argv,
    )
    (args.output_dir / "source_manifest.json").write_text(json.dumps(metadata, indent=2, default=str) + "\n")


def run_full(args):
    """Run contact-only box pickup, wheel transport, shelf return and checkout."""
    import numpy as np
    from scipy.spatial.transform import Rotation

    from isaaclab_arena_examples.library.asset_materials import asset_vertex_colors
    from isaaclab_arena_examples.library.crowded_capture import GenericCapture
    from isaaclab_arena_examples.library.dual_spatula_probe import pose
    from isaaclab_arena_examples.library.s1_library_diagnostics import (
        LibraryMotionAudit,
        create_contact_diagnostics,
        read_contact_materials,
    )

    sim, robot, books, tools, q, ik, metadata = create_full_scene(args)
    metadata["contact_materials"] = read_contact_materials(sim.stage)
    snapshot_sources(args)
    target_names = list(metadata["catalog"])
    wheels = WheelController(
        robot, args.travel_speed, args.travel_acceleration, args.wheel_speed_limit, args.arrival_gain
    )
    capture = GenericCapture(
        sim,
        dict(S1=robot, **{s: t["tool"] for s, t in tools.items()}),
        books,
        args.output_dir,
        20,
        color_reader=asset_vertex_colors,
    )
    subscription, clock, contacts = create_contact_diagnostics(args.dt)
    motion_audit = LibraryMotionAudit(books, tools, metadata["catalog"], args.dt, args.diagnostic_phase)
    args.output_dir.mkdir(exist_ok=True, parents=True)
    (args.output_dir / "simulation_source.py").write_text(Path(__file__).read_text())
    expected_plan = full_plan(args.batch_books, args.dense_shelves, args.motion_speed, args.packed_shelves)
    phases = expected_plan.copy()
    if args.probe_shelf:
        names = [p["name"] for p in phases]
        phases = [phases[0]] + phases[names.index("blocker_approach") : names.index("right_stow") + 1]
    if args.probe_high:
        names = [p["name"] for p in phases]
        phases = [phases[0]] + phases[names.index("loan_approach") :]
    if args.stop_after:
        names = [p["name"] for p in phases]
        assert args.stop_after in names
        phases = phases[: names.index(args.stop_after) + 1]
    base_goal = np.array([0.30, 0]) if args.probe_shelf else np.array([-0.18, -2.31])
    if args.probe_high:
        base_goal = np.array([0.30, 1.80])
    lift = 0.65 if args.probe_high else 0.25
    step = 0
    trace = []
    events = []
    jaw_events = []
    error = None
    initial_neighbors = {name: pose(b) for name, b in books.items() if name.startswith("Shelf")}
    max_neighbors = 0
    try:
        for phase in phases:
            name = phase["name"]
            print("[mobile] " + name, flush=True)
            moving = phase["base"] is not None
            base_pose = robot.data.root_link_pose_w.torch[0].numpy()
            base_rotation = Rotation.from_quat(base_pose[3:])
            start_lift = lift
            validate_transport_clearance(books, tools, name)
            validate_release_support(books, metadata["catalog"], phase)
            if moving:
                base_goal = np.array(phase["base"])
                duration = max(15, np.linalg.norm(base_goal - base_pose[:2]) / 0.06 + 15)
            else:
                duration = phase["duration"]
            side, action, book_name = phase["side"], phase["action"], phase["book"]
            for tool_state in tools.values():
                tool_state["phase"] = name
            active_side = side if action == "close" else "right"
            jaw = tools[active_side]["tool"]
            jaw_ids = [jaw.joint_names.index(n) for n in ("jaw_left", "jaw_right")]
            closing_start = float(np.mean(np.abs(jaw.data.joint_pos.torch[0, jaw_ids].numpy())))
            prepare_full_phase(robot, tools, books, phase, lift)
            set_grasp_force_limit(tools, phase, args.jaw_force, args.loan_jaw_force)
            quiet = 0
            grasp_quiet = 0
            closing_duration = max(
                args.jaw_close_duration, args.blocker_close_duration if book_name == "blocker" else 0
            )
            nominal_count = round(duration / args.dt)
            wait_duration = args.max_jaw_wait if args.adaptive_grasp_wait and action in ("close", "open") else 0
            count = round(max(duration, wait_duration) / args.dt)
            for k in range(count):
                f = min(1, (k + 1) / nominal_count)
                f = f * f * (3 - 2 * f)
                if phase["lift"] is not None:
                    current_lift = start_lift + (phase["lift"] - start_lift) * f
                else:
                    current_lift = lift
                base_pose = robot.data.root_link_pose_w.torch[0].numpy()
                base_rotation = Rotation.from_quat(base_pose[3:])
                for t in tools.values():
                    if moving:
                        t["goal"] = base_pose[:3] + base_rotation.apply(t["relative"])
                        t["rotation"] = base_rotation * t["relative_rotation"]
                    else:
                        t["goal"] = t["start"] * (1 - f) + t["end"] * f
                        t["rotation"] = t["slerp"](f)
                if action == "close" and closing_duration > 0:
                    closing = min(1, (k + 1) * args.dt / closing_duration)
                    closing = closing**3 * (10 - 15 * closing + 6 * closing**2)
                    tools[active_side]["opening"] = closing_start + (0.015 - closing_start) * closing
                control_tools(
                    robot,
                    tools,
                    q,
                    ik,
                    current_lift,
                    step,
                    args.dt,
                    args.arm_control_period,
                )
                wheels.command(base_goal, q, args.dt)
                clock[0] = step + 1
                sim.step(render=False)
                robot.update(args.dt)
                if step == 0 and args.cylinder_wheels:
                    measure_round_wheels(sim, robot, args.output_dir)
                for b in books.values():
                    b.update(args.dt)
                for t in tools.values():
                    t["tool"].update(args.dt)
                    for sensor in t["sensors"]:
                        sensor.update(args.dt)
                motion_audit.update(step + 1, name)
                max_neighbors = motion_audit.max_neighbor_displacement_m
                if step % capture.stride == 0:
                    base_pose = robot.data.root_link_pose_w.torch[0].numpy()
                    tool_records = {}
                    for s, t in tools.items():
                        matrix = [sensor.data.force_matrix_w.torch[0, 0].numpy() for sensor in t["sensors"]]
                        forces = {n: [float(np.linalg.norm(m[i])) for m in matrix] for i, n in enumerate(target_names)}
                        tool_records[s] = dict(
                            pose=pose(t["tool"]).tolist(),
                            held=t["held"],
                            active=t["active"],
                            forces=forces,
                            jaw_positions=t["tool"].data.joint_pos.torch[0].tolist(),
                            jaw_force_limit_N=t["tool"].data.joint_effort_limits.torch[0].tolist(),
                            arm_joint_positions=robot.data.joint_pos.torch[0, t["arm_joint_ids"]].tolist(),
                            arm_joint_velocities=robot.data.joint_vel.torch[0, t["arm_joint_ids"]].tolist(),
                            arm_joint_targets=q[0, t["arm_joint_ids"]].tolist(),
                        )
                    neighbor = max(np.linalg.norm(pose(books[n])[:3] - p[:3]) for n, p in initial_neighbors.items())
                    max_neighbors = max(max_neighbors, float(neighbor))
                    record = dict(
                        time_s=(step + 1) * args.dt,
                        phase=name,
                        base_pose=base_pose.tolist(),
                        base_velocity=robot.data.root_com_vel_w.torch[0].tolist(),
                        book_velocities={n: books[n].data.root_com_vel_w.torch[0].tolist() for n in target_names},
                        books={n: pose(books[n]).tolist() for n in target_names},
                        tools=tool_records,
                        max_neighbor_displacement_m=float(neighbor),
                    )
                    trace.append(record)
                    capture.record(step, name, record, time_s=record["time_s"])
                    if step % round(5 / args.dt) == 0:
                        print(
                            f"[mobile-state] t={step*args.dt:.1f} {name} base={base_pose[:2].tolist()} "
                            f"yaw={base_rotation.as_euler('xyz')[2]:.4f} "
                            f"return_z={record['books']['return_book'][2]:.4f}",
                            flush=True,
                        )
                step += 1
                if args.adaptive_grasp_wait and action in ("close", "open"):
                    ready = jaw_motion_settled(
                        tools[side], books[book_name], action, target_names.index(book_name), args.dt
                    )
                    grasp_quiet = grasp_quiet + args.dt if ready else 0
                    closing_complete = action != "close" or (k + 1) * args.dt >= closing_duration
                    if closing_complete and (k + 1) * args.dt >= 0.75 and grasp_quiet >= 0.5:
                        break
                if moving:
                    distance = np.linalg.norm(robot.data.root_link_pose_w.torch[0, :2].numpy() - base_goal)
                    speed = np.linalg.norm(robot.data.root_com_vel_w.torch[0, :2].numpy())
                    quiet = quiet + args.dt if distance < 0.006 and speed < 0.015 else 0
                    if quiet > 0.7:
                        break
            record_jaw_settling(
                phase, (k + 1) * args.dt, grasp_quiet, closing_duration, jaw_events, args.adaptive_grasp_wait
            )
            if moving:
                events.append(dict(phase=name, base_pose=pose(robot).tolist(), error_m=float(distance), quiet_s=quiet))
                assert quiet > 0.7, f"Wheel navigation did not settle: {name}, error={distance}, speed={speed}"
            if phase["lift"] is not None:
                lift = phase["lift"]
            if action == "close":
                t = tools[side]
                index = target_names.index(book_name)
                forces = [float(np.linalg.norm(s.data.force_matrix_w.torch[0, 0, index].numpy())) for s in t["sensors"]]
                assert min(forces) > 0.05, f"No bilateral contact: {name}: {forces}"
                t["held"] = book_name
            if action == "open":
                tools[side]["held"] = None
            (args.output_dir / "progress.json").write_text(
                json.dumps(
                    dict(
                        phase=name,
                        step=step,
                        base_pose=pose(robot).tolist(),
                        books={n: pose(books[n]).tolist() for n in target_names},
                        held={s: t["held"] for s, t in tools.items()},
                    ),
                    indent=2,
                )
                + "\n"
            )
    except (AssertionError, RuntimeError) as caught:
        error = str(caught)
        print("[mobile] FAILURE " + error, flush=True)
    subscription.unsubscribe()
    final = {n: pose(books[n]).tolist() for n in target_names}
    report = dict(
        success=False,
        error=error,
        stopped_after=args.stop_after,
        probe_shelf=args.probe_shelf,
        probe_high=args.probe_high,
        physics="PhysX",
        dt=args.dt,
        motion_audit=motion_audit.summary(),
        batch_books=args.batch_books,
        expected_plan=expected_plan,
        travel_speed_m_s=args.travel_speed,
        wheel_speed_limit_rad_s=args.wheel_speed_limit,
        motion_speed_multiplier=args.motion_speed,
        travel_acceleration_m_s2=args.travel_acceleration,
        arrival_gain=args.arrival_gain,
        jaw_close_duration_s=args.jaw_close_duration,
        blocker_close_duration_s=args.blocker_close_duration,
        arm_control_diagnostics={side: tool.get("control_diagnostics", {}) for side, tool in tools.items()},
        cylinder_wheels=args.cylinder_wheels,
        adaptive_grasp_wait=args.adaptive_grasp_wait,
        arm_control_period_s=round(args.arm_control_period / args.dt) * args.dt,
        uid=os.geteuid(),
        scene=metadata,
        base_drive=(
            "Four actual steering and driving joints with wheel-floor friction; no root pose or velocity updates after"
            " initialization"
        ),
        jaw_force_limit_N=args.jaw_force,
        loan_jaw_force_limit_N=args.loan_jaw_force,
        max_jaw_wait_s=args.max_jaw_wait,
        book_grasp=(
            f"Two prismatic jaws per wrist, default {args.jaw_force:g} N, "
            f"borrowed-book override {args.loan_jaw_force} N; contact friction only; "
            "no book joints or book pose writes"
        ),
        jaw_force_limit_events={side: tool.get("force_limit_events", []) for side, tool in tools.items()},
        self_collisions=False,
        verification_status="pending",
        final_book_poses=final,
        navigation=events,
        jaw_settling=jaw_events,
        jaw_settling_criterion="Unfiltered 240 Hz measured pose differences: <0.01 m/s and <0.05 rad/s for 0.5 s",
        max_neighbor_displacement_m=max_neighbors,
        trace=trace,
        limitations=[
            "S1 self-collisions disabled due existing collision-mesh overlaps.",
            "Uncalibrated motors, friction and tool masses; rigid books and blades.",
            (
                f"Driving-wheel limit {args.wheel_speed_limit:g} rad/s; original profile is 1.5 rad/s. Higher limits"
                " are simulated assumptions, not measured hardware capability."
            ),
        ],
    )
    (args.output_dir / "motion_audit.json").write_text(json.dumps(report["motion_audit"], indent=2) + "\n")
    (args.output_dir / "diagnostic_samples.json").write_text(json.dumps(motion_audit.diagnostic_samples) + "\n")
    capture.save(report)
    (args.output_dir / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    (args.output_dir / "contact_pairs.json").write_text(json.dumps(contacts, indent=2) + "\n")
    from isaaclab_arena_examples.library.analyze_mobile_spatula import analyze

    verified = analyze(args.output_dir, finalize=True)
    report.update(success=verified["success"], verification_status="complete")
    print(json.dumps({k: v for k, v in report.items() if k != "trace"}, indent=2), flush=True)


def main():
    """Launch the existing non-root physics runtime."""
    args = parse_args()
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

    with SimulationAppContext(args):
        if args.probe_drive:
            drive_probe(args)
        else:
            run_full(args)


if __name__ == "__main__":
    main()
