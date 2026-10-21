# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Verify official Galbot S1 lift and dual-arm control before library integration."""

import argparse
import json
from pathlib import Path

from isaaclab_arena_examples.library.galbot_s1_robot import (
    ASSET_REVISION,
    LEFT_ARM_JOINTS,
    LIFT_JOINTS,
    RIGHT_ARM_JOINTS,
    TCP_BODIES,
    TORSO_BODY,
    make_galbot_cfg,
    smooth_fraction,
)


def parse_args():
    """Parse the control-bench arguments before launching Isaac Sim."""
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--no-capture", action="store_true")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.device != "cpu":
        parser.error("Use --device cpu for this control bench.")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error("Choose a fresh output directory.")
    if args.visualizer is None:
        args.visualizer = [] if args.headless else ["kit"]
    return args


def create_scene():
    """Load the official S1 with a visual floor and fixed chassis."""
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 240, device="cpu", use_fabric=False))
    light = sim_utils.DomeLightCfg(intensity=2500)
    light.func("/World/Light", light)
    floor = sim_utils.CuboidCfg(
        size=(4.0, 4.0, 0.05),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.48, 0.49, 0.46)),
    )
    floor.func("/World/Floor", floor, translation=(0.0, 0.0, -0.056))
    cfg = make_galbot_cfg()
    robot = Articulation(cfg)
    sim.reset()
    q = robot.data.default_joint_pos.torch.clone()
    robot.write_joint_position_to_sim_index(position=q)
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q))
    robot.set_joint_position_target_index(target=q)
    sim.forward()
    robot.update(sim.get_physics_dt())
    required = (*LIFT_JOINTS, *LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS)
    assert all(name in robot.joint_names for name in required), "Official Galbot S1 joint layout changed."
    assert all(name in robot.body_names for name in (*TCP_BODIES.values(), TORSO_BODY,
                                                                 "left_hand_base_link", "right_hand_base_link"))
    sim.initialize_visualizers()
    return sim, robot, cfg


def run(args, app):
    """Exercise smooth elevation and both arms and record measured tracking checks."""
    import torch
    from pxr import Usd, UsdPhysics
    from isaaclab_arena_examples.library.crowded_capture import GenericCapture

    sim, robot, cfg = create_scene()
    initial = robot.data.default_joint_pos.torch.clone()
    limits = robot.data.soft_joint_pos_limits.torch
    index = {name: robot.joint_names.index(name) for name in robot.joint_names}
    body_index = {name: robot.body_names.index(name) for name in (*TCP_BODIES.values(), TORSO_BODY)}
    controlled = [i for i, name in enumerate(robot.joint_names)
                  if "passive" not in name and "active_joint2" not in name and not name.startswith(("left_hand_", "right_hand_"))]
    # Raise the upper body using the official prismatic lift.
    raised = initial.clone()
    raised[0, index["torso_lift_joint1"]] += 0.20
    left = raised.clone()
    left[0, index["left_arm_joint4"]] -= 0.18
    both = left.clone()
    both[0, index["right_arm_joint4"]] -= 0.18
    closed = both.clone()
    hand_indices = [i for i, name in enumerate(robot.joint_names)
                    if name.startswith(("Left_Hand_", "Right_Hand_")) and "Spread" not in name]
    for i in hand_indices:
        closed[0, i] = 0.35 * limits[0, i, 1]
    phases = [("home", initial), ("lift_torso", raised), ("move_left_arm", left),
              ("move_both_arms", both), ("close_hands", closed), ("open_hands", both),
              ("lower_torso_and_return", initial)]
    for _, target in phases:
        assert bool(((target >= limits[..., 0]) & (target <= limits[..., 1])).all()), "Joint target outside limits."
    from isaaclab_arena_examples.library.galbot_s1_materials import vertex_colors

    capture = None if args.no_capture else GenericCapture(
        sim, robot, {}, args.output_dir, 30, color_reader=vertex_colors
    )
    dt = sim.get_physics_dt()
    records = {}
    report = {"success": False, "model": "Galbot S1", "asset_revision": ASSET_REVISION, "usd_path": cfg.spawn.usd_path,
              "scope": "fixed-chassis forward posture, elevation, dual arms and five-finger hands; no library task",
              "limitations": ["fixed chassis; wheel drive not exercised", "self collisions disabled", "gravity disabled",
                              "five-finger hand opening/closing without objects; book manipulation not integrated"],
              "gravity_enabled": False, "self_collisions_enabled": False,
              "hands": "official Schunk SVH left and right; custom simulation mounting",
              "torso_body": TORSO_BODY, "joint_names": robot.joint_names, "body_names": robot.body_names,
              "wheel_joint_names": [name for name in robot.joint_names if "wheel" in name],
              "joint_types": {p.GetName(): p.GetTypeName() for p in Usd.PrimRange(sim.stage.GetPrimAtPath(cfg.prim_path))
                              if p.IsA(UsdPhysics.Joint)}}
    step = 0
    start = initial.clone()
    try:
        with torch.inference_mode():
            for phase, target in phases:
                duration = 2.0 if phase == "home" else 4.0
                quiet = 0.0
                peak_velocity = 0.0
                peak_acceleration = 0.0
                previous_velocity = robot.data.joint_vel.torch.clone()
                for local_step in range(round((duration + 2.0) / dt)):
                    assert app.is_running(), "Simulation closed."
                    command = start + (target - start) * smooth_fraction((local_step + 1) * dt / duration)
                    robot.set_joint_position_target_index(target=command)
                    robot.write_data_to_sim()
                    sim.step()
                    robot.update(dt)
                    step += 1
                    error = float((robot.data.joint_pos.torch[:, controlled] - target[:, controlled]).abs().max())
                    velocity = robot.data.joint_vel.torch.clone()
                    speed = float(velocity.abs().max())
                    peak_velocity = max(peak_velocity, speed)
                    peak_acceleration = max(peak_acceleration, float(((velocity - previous_velocity) / dt).abs().max()))
                    previous_velocity = velocity
                    quiet = quiet + dt if error < 0.015 and speed < 0.03 else 0.0
                    if capture is not None:
                        capture.record(step, phase, {"target_joint_error": error})
                records[phase] = {"joint_positions": robot.data.joint_pos.torch[0].tolist(),
                    "target_joint_positions": target[0].tolist(), "max_joint_error": error,
                    "joint_velocities": robot.data.joint_vel.torch[0].tolist(),
                    "continuous_quiet_seconds": quiet, "peak_joint_velocity": peak_velocity,
                    "peak_joint_acceleration": peak_acceleration,
                    "bodies": {name: robot.data.body_link_pose_w.torch[0, i].tolist() for name, i in body_index.items()}}
                print("FINAL_VELOCITIES", phase, {n: float(v) for n, v in zip(robot.joint_names, velocity[0]) if abs(float(v)) > 0.01}, flush=True)
                assert quiet >= 0.5, f"Tracking failed in {phase}: error={error:.4f}, quiet={quiet:.3f}"
                start = target.clone()
                print(f"[galbot] {phase}: error={error:.5f}", flush=True)
        torso_before = records["home"]["bodies"][TORSO_BODY]
        torso_after = records["lift_torso"]["bodies"][TORSO_BODY]
        elevation = abs(torso_after[2] - torso_before[2])
        arm_motion = {}
        for side, first, second in (("left", "lift_torso", "move_left_arm"),
                                    ("right", "move_left_arm", "move_both_arms")):
            a = torch.tensor(records[first]["bodies"][TCP_BODIES[side]][:3])
            b = torch.tensor(records[second]["bodies"][TCP_BODIES[side]][:3])
            arm_motion[side] = float((a - b).norm())
        finger_excursion = min(abs(records["close_hands"]["joint_positions"][i]
                                   - records["open_hands"]["joint_positions"][i]) for i in hand_indices)
        checks = {"both_hands_open_and_close": finger_excursion > 0.10,"elevation_over_10cm": elevation > 0.10,
                  "both_wrists_move_over_2cm": all(value > 0.02 for value in arm_motion.values()),
                  "continuous_settling": all(record["continuous_quiet_seconds"] >= 0.5 for record in records.values())}
        report.update(
            success=all(checks.values()), checks=checks, min_finger_excursion_rad=finger_excursion, elevation_m=elevation, wrist_displacement_m=arm_motion,
            reason="control_bringup_completed" if all(checks.values()) else "motion_validation_failed",
            command_interpolation="quintic with zero endpoint velocity and acceleration",
        )
    except Exception as exc:
        report["reason"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        report.update(steps=step, records=records)
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        if capture is not None:
            capture.record(step, phase, force=True)
            capture.save(report)
    assert report["success"], report
    print(json.dumps({"success": report["success"], "elevation_m": report["elevation_m"]}), flush=True)


def main():
    """Run the official-asset bring-up in the existing Isaac Sim runtime."""
    args = parse_args()
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

    with SimulationAppContext(args) as app:
        run(args, app)


if __name__ == "__main__":
    main()
