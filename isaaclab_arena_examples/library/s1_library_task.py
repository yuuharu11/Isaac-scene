# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Run Galbot S1 returns, shelf rearrangement and checkout with articulated SVH hands."""

import argparse
import json
import math
from pathlib import Path

from isaaclab_arena_examples.library.galbot_s1_robot import make_galbot_cfg, smooth_fraction
from isaaclab_arena_examples.library.s1_library_scene import CATALOG, STATIONS, create_library


BOOK_FROM_WRIST = {"left": (0.35, -0.035, -0.02), "right": (0.35, 0.035, -0.02)}

def parse_args():
    """Parse the standalone library demo options before launching Isaac Sim."""
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--probe-pick", action="store_true", help="Stop after physically lifting the first book.")
    parser.add_argument("--probe-shelf", action="store_true", help="Test the shelf-side brown-book shift and gap opening.")
    parser.add_argument("--probe-high", action="store_true", help="Test high-shelf retrieval through lowering for travel.")
    parser.add_argument("--no-capture", action="store_true")
    parser.add_argument("--capture-fps", type=float, default=15.0)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    assert sum((args.probe_pick, args.probe_shelf, args.probe_high)) <= 1, "Select only one probe mode."
    assert args.device == "cpu", "Use --device cpu."
    assert not args.output_dir.exists() or not any(args.output_dir.iterdir()), "Choose a fresh output directory."
    args.headless = True
    args.visualizer = []
    return args


def spawn_library_robot(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Spawn articulated SVH hands with contact-free fingers for the ideal-grasp task.

    Args:
        prim_path: Robot root path.
        cfg: S1 USD spawn configuration.
        translation: Chassis position.
        orientation: Chassis quaternion.
        **kwargs: Forwarded spawner options.

    Returns:
        The spawned S1 prim; arm and chassis colliders remain enabled.
    """
    from pxr import Usd, UsdPhysics
    from isaaclab_arena_examples.library.galbot_s1_hands import spawn_s1_with_hands

    root = spawn_s1_with_hands(prim_path, cfg, translation, orientation, **kwargs)
    for side in ("left", "right"):
        hand = root.GetStage().GetPrimAtPath(f"{prim_path}/{side}_svh")
        for prim in Usd.PrimRange(hand):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
    return root


def create_scene(station="boxes"):
    """Create the S1 library before running trajectories.

    Args:
        station: Initial chassis station.

    Returns:
        Simulation, robot, target books, holding joints and scene metadata.
    """
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from pxr import Usd, UsdPhysics

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 240, device="cpu", use_fabric=False))
    light = sim_utils.DomeLightCfg(intensity=2500)
    light.func("/World/Light", light)
    books, metadata = create_library(sim)
    cfg = make_galbot_cfg(position=STATIONS[station])
    cfg.spawn.func = spawn_library_robot
    robot = Articulation(cfg)
    joints = {}
    rigid = [p.GetPath() for p in Usd.PrimRange(sim.stage.GetPrimAtPath(cfg.prim_path)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for name, side in (("return_book", "left"), ("blocker", "right"), ("loan_book", "right")):
        joint = UsdPhysics.FixedJoint.Define(sim.stage, "/World/Hold_" + name)
        joint.CreateBody0Rel().SetTargets([f"{cfg.prim_path}/{side}_svh/{side}_hand_base_link"])
        joint.CreateBody1Rel().SetTargets([books[name].cfg.prim_path])
        joint.CreateJointEnabledAttr(False)
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateCollisionEnabledAttr(False)
        joints[name] = joint
        # Ideal holding is deliberate: fingers animate, target-book/robot collisions are filtered.
        pairs = UsdPhysics.FilteredPairsAPI.Apply(sim.stage.GetPrimAtPath(books[name].cfg.prim_path))
        pairs.CreateFilteredPairsRel().SetTargets(rigid)
    # Collision groups cover merged articulation links as well as individual rigid prims.
    robot_group = UsdPhysics.CollisionGroup.Define(sim.stage, "/World/RobotCollisionGroup")
    book_group = UsdPhysics.CollisionGroup.Define(sim.stage, "/World/TargetBookCollisionGroup")
    robot_group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([cfg.prim_path])
    book_group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([book.cfg.prim_path for book in books.values()])
    robot_group.CreateFilteredGroupsRel().SetTargets([book_group.GetPath()])
    book_group.CreateFilteredGroupsRel().SetTargets([robot_group.GetPath()])
    sim.reset()
    q = robot.data.default_joint_pos.torch.clone()
    robot.write_joint_position_to_sim_index(position=q)
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q))
    robot.set_joint_position_target_index(target=q)
    sim.forward()
    for obj in (robot, *books.values()):
        obj.update(sim.get_physics_dt())
    sim.initialize_visualizers()
    return sim, robot, books, joints, metadata


class LibraryTask:
    """Execute measured joint trajectories, catalog waypoints and reversible ideal grasps."""

    def __init__(self, sim, robot, books, joints):
        from isaaclab_arena_examples.library.s1_library_ik import S1Kinematics

        self.sim, self.robot, self.books, self.joints = sim, robot, books, joints
        self.kinematics = S1Kinematics()
        self.names = {n: i for i, n in enumerate(robot.joint_names)}
        self.arm_ids = {s: [self.names[f"{s}_arm_joint{i}"] for i in range(1, 8)] for s in ("left", "right")}
        self.body_ids = {s: robot.body_names.index(s + "_arm_link7") for s in ("left", "right")}
        self.drive_ids = [i for i, n in enumerate(robot.joint_names) if not n.startswith(("left_hand_", "right_hand_"))]
        self.q = robot.data.joint_pos.torch.clone()
        self.base = robot.data.root_pose_w.torch[0, :3].tolist()
        self.lift = 0.25
        self.events, self.released, self.held = [], set(), {}
        self.distance = 0.0
        self.phases = self.make_plan()
        self.executed = []

    def make_plan(self):
        """Return collision-aware approach, extraction and placement waypoints."""
        plan = []
        def move(name, side=None, book=None, xyz=None, action=None, base=None, lift=None):
            plan.append(dict(name=name, side=side, book=book, xyz=xyz, action=action, base=base, lift=lift))
        move("settle")
        move("return_approach", "left", "return_book", (0.78, -2.10, 1.10))
        move("return_reach", "left", "return_book", CATALOG["return_book"]["source"])
        move("grip_return", "left", "return_book", action="grasp")
        move("lift_return", "left", "return_book", (0.85, -2.10, 1.20))
        move("stow_return", "left", "return_book", (0.85, -2.06, 1.05))
        move("drive_shelf_a", base=STATIONS["shelf_a"])
        move("catalog_return", action="catalog_return")
        move("blocker_approach", "right", "blocker", (1.15, 0.0, 1.02))
        move("blocker_reach", "right", "blocker", CATALOG["blocker"]["source"])
        move("grip_blocker", "right", "blocker", action="grasp")
        move("blocker_raise", "right", "blocker", (1.35, 0.0, 1.02))
        move("blocker_park", "right", "blocker", (1.35, -0.15, 1.02), "gap_check")
        move("blocker_seat", "right", "blocker", (1.35, -0.15, 0.9715))
        move("release_blocker", "right", "blocker", action="release")
        move("blocker_withdraw", "right", "blocker", (1.08, -0.15, 1.06))
        move("right_stow", "right", "blocker", (1.08, -0.25, 1.05))
        move("return_align", "left", "return_book", (1.15, 0.0, 1.02))
        move("return_insert", "left", "return_book", (1.35, 0.0, 1.02))
        move("return_seat", "left", "return_book", (1.35, 0.0, 0.9715))
        move("release_return", "left", "return_book", action="release")
        move("return_withdraw", "left", "return_book", (1.12, 0.0, 1.06))
        move("left_stow", "left", "return_book", (1.08, 0.25, 1.05))
        move("drive_shelf_b", base=STATIONS["shelf_b"])
        move("raise_for_shelf_b", lift=0.65)
        move("catalog_loan", action="catalog_loan")
        move("loan_approach", "right", "loan_book", (1.15, 1.59, 1.68))
        move("loan_reach", "right", "loan_book", CATALOG["loan_book"]["source"])
        move("grip_loan", "right", "loan_book", action="grasp")
        move("loan_raise", "right", "loan_book", (1.35, 1.59, 1.68))
        move("loan_extract", "right", "loan_book", (1.10, 1.59, 1.68))
        move("loan_clear_shelf", "right", "loan_book", (1.04, 1.59, 1.68))
        move("loan_stow", "right", "loan_book", (1.04, 1.55, 1.50))
        move("left_high_stow", "left", xyz=(1.08, 2.05, 1.45))
        move("lower_for_travel", lift=0.25)
        move("drive_checkout", base=(0.12, -1.345, 0.0))
        move("checkout_approach", "right", "loan_book", (0.85, -1.55, 1.12))
        move("checkout_seat", "right", "loan_book", (0.85, -1.55, 0.9715))
        move("release_loan", "right", "loan_book", action="release")
        move("checkout_clear", "right", "loan_book", (0.72, -1.55, 1.15))
        move("done")
        return plan

    def prepare(self, phase):
        """Solve the next bounded arm target before moving.

        Args:
            phase: One catalog waypoint or control action.

        Returns:
            Joint targets, segment duration and optional world wrist target.
        """
        import numpy as np
        import torch

        target = self.q.clone()
        side, xyz = phase["side"], phase["xyz"]
        expected = None
        if xyz is not None:
            # Book centre relative to wrist: spine fingertip grip, palm kept outside the shelf.
            offset = np.array(BOOK_FROM_WRIST[side])
            wrist = np.array(xyz) - offset
            local = wrist - np.array(self.base)
            seed = self.q[0, self.arm_ids[side]].cpu().numpy()
            values, error = self.kinematics.solve(side, local, self.lift, seed)
            target[0, self.arm_ids[side]] = torch.tensor(values, dtype=target.dtype)
            expected = wrist.tolist()
        if phase["lift"] is not None:
            target[0, self.names["torso_lift_joint1"]] = phase["lift"]
        if phase["action"] in ("grasp", "release"):
            limits = self.robot.data.soft_joint_pos_limits.torch
            for name, i in self.names.items():
                if name.startswith(side.capitalize() + "_Hand_") and "Spread" not in name:
                    target[0, i] = 0.35 * limits[0, i, 1] if phase["action"] == "grasp" else 0.0
        limits = self.robot.data.soft_joint_pos_limits.torch
        assert bool(((target[:, self.drive_ids] >= limits[:, self.drive_ids, 0]) &
                     (target[:, self.drive_ids] <= limits[:, self.drive_ids, 1])).all()), phase["name"]
        duration = max(1.5, 1.875 * float((target - self.q).abs().max()) / 0.65)
        if phase["base"] is not None:
            distance = math.dist(self.base, phase["base"])
            duration = max(2.0, distance / 0.22)
        return target, duration, expected

    def action(self, phase):
        """Attach or release books only after measured arrival at the commanded pose."""
        from pxr import Gf
        from isaaclab.utils.math import subtract_frame_transforms

        action, side, name = phase["action"], phase["side"], phase["book"]
        if action == "grasp":
            book = self.books[name].data.root_pose_w.torch
            wrist = self.robot.data.body_link_pose_w.torch[:, self.body_ids[side]]
            desired = wrist[:, :3] + wrist.new_tensor([BOOK_FROM_WRIST[side]])
            error = float((desired - book[:, :3]).norm())
            assert error < 0.018, f"Book moved before grasp: {name}, error={error:.4f}"
            body = self.robot.body_names.index(side + "_hand_base_link")
            hand = self.robot.data.body_link_pose_w.torch[:, body]
            p, q = subtract_frame_transforms(hand[:, :3], hand[:, 3:], book[:, :3], book[:, 3:])
            joint = self.joints[name]
            joint.CreateLocalPos0Attr(Gf.Vec3f(*p[0].tolist()))
            quat = q[0].tolist()
            joint.CreateLocalRot0Attr(Gf.Quatf(quat[3], Gf.Vec3f(*quat[:3])))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
            joint.CreateLocalRot1Attr(Gf.Quatf(1.0))
            joint.GetJointEnabledAttr().Set(True)
            self.held[side] = name
            self.events.append(dict(event="grasp", book=name, side=side, height_m=float(book[0, 2]), position_error_m=error))
        elif action == "release":
            assert self.held.get(side) == name
            self.joints[name].GetJointEnabledAttr().Set(False)
            self.held.pop(side)
            self.released.add(name)
            self.events.append(dict(event="release", book=name, side=side))
        elif action == "gap_check":
            pos = self.books["blocker"].data.root_pose_w.torch[0, :3].tolist()
            free_width = 0.05 - pos[1] - 0.035
            assert free_width > 0.15, "The gap between brown and green is too narrow."
            self.events.append(dict(event="slot_cleared", slot="A-01", free_width_m=free_width))
        elif action and action.startswith("catalog_"):
            key = "return_book" if action == "catalog_return" else "loan_book"
            self.events.append(dict(event="catalog_match", book=key, catalog_id=CATALOG[key]["id"], slot=CATALOG[key]["slot"]))


def run(args, app):
    """Execute the library workflow and save measured success criteria and captures."""
    import torch
    from isaaclab_arena_examples.library.crowded_capture import GenericCapture
    from isaaclab_arena_examples.library.galbot_s1_materials import vertex_colors

    sim, robot, books, joints, metadata = create_scene("shelf_b" if args.probe_high else "shelf_a" if args.probe_shelf else "boxes")
    ctrl = LibraryTask(sim, robot, books, joints)
    if args.probe_shelf:
        names = ("settle", "blocker_approach", "blocker_reach", "grip_blocker", "blocker_raise", "blocker_park", "blocker_seat", "release_blocker", "blocker_withdraw", "right_stow")
        ctrl.phases = [p for p in ctrl.phases if p["name"] in names]
    if args.probe_high:
        names = ("settle", "raise_for_shelf_b", "catalog_loan", "loan_approach", "loan_reach", "grip_loan", "loan_raise", "loan_extract", "loan_clear_shelf", "loan_stow", "left_high_stow", "lower_for_travel")
        ctrl.phases = [p for p in ctrl.phases if p["name"] in names]
    capture = None if args.no_capture else GenericCapture(sim, robot, books, args.output_dir, args.capture_fps, color_reader=vertex_colors)
    report = dict(success=False, scene=metadata, events=ctrl.events,
                  scope="S1 library prototype with articulated hands and dynamic target books",
                  limitations=["prescribed kinematic base route; no traction/SLAM/planning",
                               "ideal fixed-joint grasps and robot-target collision filtering; not friction grasping",
                               "robot gravity and self collisions disabled; books retain gravity",
                               "hand colliders disabled for ideal grasps; robot body and book/furniture collisions retained",
                               "fixed shelf-neighbor books; no perception or fallen-book recovery"])
    dt, step, phase_name = sim.get_physics_dt(), 0, "settle"
    try:
        with torch.inference_mode():
            for phase in ctrl.phases:
                assert app.is_running()
                phase_name = phase["name"]
                target, duration, expected = ctrl.prepare(phase)
                start, start_base = ctrl.q.clone(), ctrl.base.copy()
                goal_base = phase["base"] or start_base
                quiet, wrist_error = 0.0, 0.0
                print("[s1-library] " + phase_name, flush=True)
                for local in range(round((duration + 3.0) / dt)):
                    weight = smooth_fraction((local + 1) * dt / duration)
                    command = start + (target - start) * weight
                    robot.set_joint_position_target_index(target=command)
                    if phase["base"] is not None:
                        position = [a + (b - a) * weight for a, b in zip(start_base, goal_base)]
                        root = robot.data.root_pose_w.torch.clone()
                        root[0, :3] = torch.tensor(position)
                        robot.write_root_pose_to_sim_index(root_pose=root)
                    robot.write_data_to_sim()
                    sim.step()
                    for obj in (robot, *books.values()):
                        obj.update(dt)
                    step += 1
                    error = float((robot.data.joint_pos.torch[:, ctrl.drive_ids] - target[:, ctrl.drive_ids]).abs().max())
                    speed = float(robot.data.joint_vel.torch.abs().max())
                    if expected is not None:
                        actual = robot.data.body_link_pose_w.torch[0, ctrl.body_ids[phase["side"]], :3]
                        wrist_error = float((actual - torch.tensor(expected)).norm())
                    quiet = quiet + dt if (local + 1) * dt >= duration and error < 0.025 and speed < 0.05 and wrist_error < 0.015 else 0.0
                    if capture:
                        capture.record(step, phase_name, dict(joint_error=error, wrist_error=wrist_error))
                    if quiet >= 0.6:
                        break
                assert quiet >= 0.6, f"Tracking failed: {phase_name}, q={error:.4f}, speed={speed:.4f}, wrist={wrist_error:.4f}"
                ctrl.q = target
                if phase["base"] is not None:
                    actual_base = robot.data.root_pose_w.torch[0, :3].tolist()
                    assert math.dist(actual_base, goal_base) < 0.002, "Chassis failed to reach its station."
                    ctrl.distance += math.dist(start_base, goal_base)
                    ctrl.base = list(goal_base)
                    ctrl.events.append(dict(event="station_arrived", phase=phase_name, base=ctrl.base.copy()))
                if phase["lift"] is not None:
                    ctrl.lift = phase["lift"]
                ctrl.action(phase)
                ctrl.executed.append(dict(phase=phase_name, max_joint_error=error, wrist_error_m=wrist_error,
                                          quiet_seconds=quiet, base=ctrl.base.copy(),
                                          lift_position_m=float(robot.data.joint_pos.torch[0, ctrl.names["torso_lift_joint1"]])))
                if args.probe_high and phase_name == "lower_for_travel":
                    assert ctrl.held.get("right") == "loan_book"
                    report.update(success=True, reason="representative_high_probe")
                    break
                if args.probe_shelf and phase_name == "right_stow":
                    assert "blocker" in ctrl.released and "right" not in ctrl.held
                    report.update(success=True, reason="representative_shelf_probe")
                    break
                if args.probe_pick and phase_name == "lift_return":
                    book_z = float(books["return_book"].data.root_pose_w.torch[0, 2])
                    assert book_z > 1.15 and ctrl.held.get("left") == "return_book"
                    report.update(success=True, reason="representative_pick_probe", lifted_book_z=book_z)
                    break
            else:
                checks = {}
                for name, obj in books.items():
                    pose = obj.data.root_pose_w.torch[0]
                    velocity = float(obj.data.root_vel_w.torch.norm())
                    error = math.dist(pose[:3].tolist(), CATALOG[name]["target"])
                    upright = 1 - 2 * float((pose[3:5] ** 2).sum())
                    checks[name] = error < 0.025 and velocity < 0.10 and upright > math.cos(math.radians(15)) and name in ctrl.released
                checks["all_released"] = not ctrl.held
                checks["slot_cleared_before_insertion"] = any(e["event"] == "slot_cleared" for e in ctrl.events)
                checks["compact_route"] = 7.0 < ctrl.distance < 7.5
                checks["neighbor_shifted_right_15cm"] = math.dist(books["blocker"].data.root_pose_w.torch[0, :3].tolist(), CATALOG["blocker"]["target"]) < 0.005
                checks["high_shelf_retrieval"] = any(e["event"] == "grasp" and e["book"] == "loan_book" and e.get("height_m", 0) > 1.60 for e in ctrl.events)
                report.update(success=all(checks.values()), reason="complete" if all(checks.values()) else "placement_failed", checks=checks)
    except Exception as exc:
        report["reason"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        report.update(steps=step, phase=phase_name, distance_m=ctrl.distance, trajectory=ctrl.executed,
                      held=ctrl.held, released=sorted(ctrl.released),
                      robot_joint_positions=robot.data.joint_pos.torch[0].tolist(),
                      robot_joint_velocities=robot.data.joint_vel.torch[0].tolist(),
                      robot_joint_names=robot.joint_names,
                      objects={n: dict(pose_xyzw=o.data.root_pose_w.torch[0].tolist(), velocity=o.data.root_vel_w.torch[0].tolist()) for n, o in books.items()})
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        if capture:
            capture.record(step, phase_name, force=True)
            capture.save(report)
    assert report["success"], report.get("reason")
    print(json.dumps({"success": report["success"], "reason": report["reason"], "distance_m": ctrl.distance}), flush=True)


def main():
    """Run the standalone S1 library demo inside the existing Isaac Sim container."""
    args = parse_args()
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

    with SimulationAppContext(args) as app:
        run(args, app)


if __name__ == "__main__":
    main()
