# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Verify official Galbot articulation control before integrating library manipulation."""

import argparse
import json
from pathlib import Path

from isaaclab_arena_examples.library.galbot_robot import (
    LEFT_ARM_JOINTS,
    LEG_JOINTS,
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
    """Load the official robot with a visual floor below its authored wheels."""
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cpu", use_fabric=False))
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
    required = (*LEG_JOINTS, *LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS)
    assert all(name in robot.joint_names for name in required), "Official Galbot joint layout changed."
    assert all(name in robot.body_names for name in (*TCP_BODIES.values(), TORSO_BODY))
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
    # Compensate the two pitch joints so torso orientation is approximately preserved.
    lowered = initial.clone()
    lowered[0, index["leg_joint1"]] -= 0.45
    lowered[0, index["leg_joint3"]] += 0.45
    left = lowered.clone()
    left[0, index["left_arm_joint4"]] -= 0.18
    both = left.clone()
    both[0, index["right_arm_joint4"]] -= 0.18
    phases = [("home", initial), ("lower_torso", lowered), ("move_left_arm", left),
              ("move_both_arms", both), ("raise_torso_and_return", initial)]
    for _, target in phases:
        assert bool(((target >= limits[..., 0]) & (target <= limits[..., 1])).all()), "Joint target outside limits."
    capture = None if args.no_capture else GenericCapture(sim, robot, {}, args.output_dir, 30)
    dt = sim.get_physics_dt()
    records = {}
    report = {"success": False, "model": "Galbot One Charlie", "usd_path": cfg.spawn.usd_path,
              "scope": "fixed-chassis elevation and dual-arm control bring-up; no library task",
              "limitations": ["self collisions disabled", "gravity disabled", "wheel drive not exercised",
                              "stock left gripper and right suction tool retained"],
              "joint_names": robot.joint_names, "body_names": robot.body_names,
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
                    error = float((robot.data.joint_pos.torch - target).abs().max())
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
                    "continuous_quiet_seconds": quiet, "peak_joint_velocity": peak_velocity,
                    "peak_joint_acceleration": peak_acceleration,
                    "bodies": {name: robot.data.body_link_pose_w.torch[0, i].tolist() for name, i in body_index.items()}}
                assert quiet >= 0.5, f"Tracking failed in {phase}: error={error:.4f}, quiet={quiet:.3f}"
                start = target.clone()
                print(f"[galbot] {phase}: error={error:.5f}", flush=True)
        torso_before = records["home"]["bodies"][TORSO_BODY]
        torso_after = records["lower_torso"]["bodies"][TORSO_BODY]
        elevation = abs(torso_after[2] - torso_before[2])
        arm_motion = {}
        for side, first, second in (("left", "lower_torso", "move_left_arm"),
                                    ("right", "move_left_arm", "move_both_arms")):
            a = torch.tensor(records[first]["bodies"][TCP_BODIES[side]][:3])
            b = torch.tensor(records[second]["bodies"][TCP_BODIES[side]][:3])
            arm_motion[side] = float((a - b).norm())
        checks = {"elevation_over_10cm": elevation > 0.10,
                  "both_tools_move_over_2cm": all(value > 0.02 for value in arm_motion.values()),
                  "continuous_settling": all(record["continuous_quiet_seconds"] >= 0.5 for record in records.values())}
        report.update(
            success=all(checks.values()), checks=checks, elevation_m=elevation, tool_displacement_m=arm_motion,
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
