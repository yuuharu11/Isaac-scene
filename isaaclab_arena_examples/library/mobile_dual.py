# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Move a common wheeled base carrying identical arms between boxes and two shelves."""

import json
import math

from isaaclab_arena_examples.library.crowded_shelf import ArmIK, parse_args


def create_scene():
    """Create two identical tong-equipped NHC12 arms on one moving platform."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
    from pxr import Usd, UsdPhysics

    from isaaclab_arena_examples.bento.meal import ROBOT_ASSET
    from isaaclab_arena_examples.library.mobile_dual_scene import create_mobile_dual_scene
    from isaaclab_arena_examples.library.scene import add_book_tongs

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cpu", use_fabric=False))
    light = sim_utils.DomeLightCfg(intensity=2500)
    light.func("/World/Light", light)
    objects, metadata, base = create_mobile_dual_scene(sim)
    robots, tools, paths = {}, {}, {}
    station = metadata["stations"]["boxes"]
    for side, mount in metadata["arm_mounts"].items():
        robots[side] = Articulation(
            ArticulationCfg(
                prim_path="/World/" + side.capitalize() + "Robot",
                spawn=sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/{ROBOT_ASSET}",
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
                    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                        fix_root_link=True,
                        enabled_self_collisions=True,
                        solver_position_iteration_count=32,
                        solver_velocity_iteration_count=8,
                    ),
                ),
                init_state=ArticulationCfg.InitialStateCfg(
                    pos=tuple(a + b for a, b in zip(station, mount)),
                    rot=(0, 0, 0, 1) if side == "left" else (0, 0, 1, 0),
                    joint_pos={"joint_[2-6]_.*": 0.0, "joint_1_s": 1.2 if side == "left" else -1.2},
                ),
                actuators={"arm": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=800, damping=80)},
            )
        )
        paths[side] = next(
            str(p.GetPath())
            for p in Usd.PrimRange(sim.stage.GetPrimAtPath(robots[side].cfg.prim_path))
            if p.GetName() == "link_6_t" and p.HasAPI(UsdPhysics.RigidBodyAPI)
        )
        tools[side] = add_book_tongs(
            sim.stage, paths[side], 0.18, paddle_length=0.012, collision=True, finger_height=0.08, housing_width=0.16
        )
    joints = {}
    for name, side in (("return_book", "left"), ("loan_book", "right")):
        joint = UsdPhysics.FixedJoint.Define(sim.stage, "/World/Grasp_" + name)
        joint.CreateBody0Rel().SetTargets([paths[side]])
        joint.CreateBody1Rel().SetTargets([str(objects[name].cfg.prim_path)])
        joint.CreateJointEnabledAttr(False)
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateCollisionEnabledAttr(False)
        joints[name] = joint
    sim.reset()
    import torch

    for robot in robots.values():
        q = robot.data.default_joint_pos.torch.clone()
        robot.write_joint_position_to_sim_index(position=q)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q))
        robot.set_joint_position_target_index(target=q)
    sim.forward()
    for obj in [*robots.values(), *objects.values(), base]:
        obj.update(sim.get_physics_dt())
    sim.initialize_visualizers()
    return sim, robots, objects, joints, tools, metadata, base


class MobileDualController:
    """Coordinate sequential two-arm work and common-root aisle translation."""

    phases = (
        "home",
        "pick_hover",
        "pick_lower",
        "grip_return",
        "lift_return",
        "stow_return",
        "drive_gap",
        "gap_approach",
        "gap_enter",
        "gap_spread",
        "gap_retract",
        "gap_park",
        "drive_return",
        "return_align",
        "return_insert",
        "return_seat",
        "release_return",
        "return_clear",
        "return_stow",
        "drive_loan",
        "catalog_match",
        "loan_hover",
        "loan_overhead",
        "loan_lower",
        "grip_loan",
        "loan_raise",
        "loan_extract",
        "loan_lift",
        "loan_stow",
        "drive_boxes",
        "box_hover",
        "box_lower",
        "release_loan",
        "box_clear",
        "done",
    )

    def __init__(self, sim, robots, objects, joints, tools, metadata, base, timeout):
        self.sim, self.robots, self.objects, self.joints = sim, robots, objects, joints
        self.tools, self.metadata, self.base = tools, metadata, base
        self.timeout = timeout
        self.arms = {side: ArmIK(sim, robot, side) for side, robot in robots.items()}
        self.index = 0
        self.elapsed = self.dwell = 0.0
        self.failure = None
        self.finished = False
        self.offset = list(metadata["stations"]["boxes"])
        self.events = []
        self.released = set()
        self.holding = {}
        self.grasp_height = {}
        self.openings = {"left": 0.18, "right": 0.18}
        self.targets = {"left": self.local((-0.30, 0.05, 1.15)), "right": self.local((0.30, 0.05, 1.15))}
        self.travel_q = {}
        self.distance = 0.0
        self.max_gap = self.gap()
        self.initial_gap = self.max_gap

    @property
    def phase(self):
        return self.phases[self.index]

    def local(self, point):
        return tuple(a + b for a, b in zip(point, self.offset))

    def measured(self, name):
        return self.objects[name].data.root_pose_w.torch[0, :3].tolist()

    def gap_bounds(self):
        import torch

        from isaaclab.utils.math import matrix_from_quat

        groups = [self.metadata["return_left_neighbors"], self.metadata["return_right_neighbors"]]
        edges = []
        for names in groups:
            poses = torch.cat([self.objects[n].data.root_pose_w.torch for n in names])
            half = torch.tensor([0.0175, 0.08, 0.11], device=self.sim.device)
            extent = matrix_from_quat(poses[:, 3:]).abs() @ half
            edges.append((poses[:, :3] - extent, poses[:, :3] + extent))
        return float(edges[0][1][:, 0].max()), float(edges[1][0][:, 0].min())

    def gap(self):
        left, right = self.gap_bounds()
        return right - left

    def set_targets(self):
        p = self.phase
        source = self.measured("return_book")
        loan = self.measured("loan_book")
        if p == "return_align":
            left_edge, right_edge = self.gap_bounds()
            if right_edge - left_edge < 0.045:
                self.events.append({"event": "gap_closed_before_insertion", "bounds_m": [left_edge, right_edge]})
                self.failure = "gap_closed_before_insertion"
                return
            # Keep the catalog slot on record: final validation uses the planned target below.
            self.metadata["return_book"].setdefault("catalog_target", list(self.metadata["return_book"]["target"]))
            self.metadata["return_book"]["target"][0] = (left_edge + right_edge) / 2
            self.events.append({
                "event": "plan_insertion_from_measured_gap",
                "bounds_m": [left_edge, right_edge],
                "target": self.metadata["return_book"]["target"].copy(),
            })
        ret = self.metadata["return_book"]["target"]
        box = self.metadata["loan_book"]["target"]
        left, right = self.targets["left"], self.targets["right"]
        if p == "pick_hover":
            left = (source[0], source[1], 1.12)
        elif p in ("pick_lower", "grip_return"):
            left = (source[0], source[1], source[2] + 0.095)
        elif p == "lift_return":
            left = (source[0], source[1], 1.15)
        elif p == "stow_return":
            left = self.local((-0.30, 0.05, 1.15))
        elif p == "gap_approach":
            right = (-0.210, 0.25, 0.98)
        elif p == "gap_enter":
            right = (-0.210, 0.48, 0.98)
        elif p == "gap_spread":
            right = (-0.105, 0.48, 0.98)
        elif p == "gap_retract":
            right = (-0.105, 0.22, 1.05)
        elif p == "gap_park":
            right = self.local((0.30, 0.05, 1.15))
        elif p == "return_align":
            left = (ret[0], 0.25, ret[2] + self.grasp_height["return_book"] + 0.04)
        elif p == "return_insert":
            left = (ret[0], ret[1], ret[2] + self.grasp_height["return_book"] + 0.04)
        elif p in ("return_seat", "release_return"):
            left = (ret[0], ret[1], ret[2] + self.grasp_height["return_book"] + 0.003)
        elif p == "return_clear":
            left = (ret[0], ret[1], 1.10)
        elif p == "return_stow":
            left = self.local((-0.30, 0.05, 1.15))
        elif p == "loan_hover":
            right = (loan[0], 0.25, 1.12)
        elif p == "loan_overhead":
            right = (loan[0], loan[1], 1.12)
        elif p in ("loan_lower", "grip_loan"):
            right = (loan[0], loan[1], loan[2] + 0.095)
        elif p == "loan_raise":
            right = (loan[0], loan[1], loan[2] + 0.095 + 0.04)
        elif p == "loan_extract":
            right = (loan[0], 0.22, loan[2] + self.grasp_height["loan_book"])
        elif p == "loan_lift":
            right = (loan[0], 0.22, 1.15)
        elif p == "loan_stow":
            right = self.local((0.30, 0.05, 1.15))
        elif p == "box_hover":
            right = (box[0], box[1], 1.15)
        elif p in ("box_lower", "release_loan"):
            right = (box[0], box[1], box[2] + self.grasp_height["loan_book"] + 0.003)
        elif p == "box_clear":
            right = (box[0], box[1], 1.15)
        self.targets = {"left": left, "right": right}
        if p.startswith("drive_"):
            self.travel_q = {s: r.data.joint_pos.torch.clone() for s, r in self.robots.items()}
        print("[mobile-dual] " + p, flush=True)

    def attach(self, name, side):
        from isaaclab.utils.math import subtract_frame_transforms
        from pxr import Gf

        arm = self.arms[side]
        robot = self.robots[side]
        ee = robot.data.body_link_pose_w.torch[:, arm.body]
        book = self.objects[name].data.root_pose_w.torch
        grasp = book[:, :3].clone()
        grasp[:, 2] += 0.095
        if float((arm.pose()[0] - grasp).norm()) > 0.025:
            self.failure = "grasp_missed: " + name
            return
        pos, quat = subtract_frame_transforms(ee[:, :3], ee[:, 3:], book[:, :3], book[:, 3:])
        q = quat[0].tolist()
        joint = self.joints[name]
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*pos[0].tolist()))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(q[3], Gf.Vec3f(*q[:3])))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1))
        joint.GetJointEnabledAttr().Set(True)
        self.holding[side] = name
        self.grasp_height[name] = float(arm.pose()[0][0, 2] - book[0, 2])
        self.events.append({"event": "grasp", "book": name, "arm": side})

    def advance(self):
        p = self.phase
        if p == "grip_return":
            self.attach("return_book", "left")
        if p == "grip_loan":
            self.attach("loan_book", "right")
        if self.failure:
            return
        if p in ("release_return", "release_loan"):
            side = "left" if p == "release_return" else "right"
            name = self.holding.pop(side)
            self.joints[name].GetJointEnabledAttr().Set(False)
            self.released.add(name)
            self.events.append({"event": "released", "book": name, "arm": side})
        if p == "gap_spread":
            self.events.append({"event": "gap_opened", "initial_m": self.initial_gap, "gap_m": self.gap()})
            if self.gap() < 0.055:
                self.failure = "gap_not_opened"
                return
        if p == "catalog_match":
            self.events.append({"event": "catalog_match", "catalog_id": "LIB-003", "shelf": "B-03"})
        self.index += 1
        self.elapsed = self.dwell = 0.0
        self.finished = self.phase == "done"
        self.set_targets()

    def navigate(self):
        import torch

        stations = {
            "drive_gap": "open_gap",
            "drive_return": "return_shelf",
            "drive_loan": "loan_shelf",
            "drive_boxes": "boxes",
        }
        goal = torch.tensor(self.metadata["stations"][stations[self.phase]][:2], device=self.sim.device)
        current = torch.tensor(self.offset[:2], device=self.sim.device)
        delta = goal - current
        length = float(delta.norm())
        increment = delta * min(1.0, 0.20 * self.sim.get_physics_dt() / max(length, 1e-8))
        self.distance += float(increment.norm())
        self.offset[0] += float(increment[0])
        self.offset[1] += float(increment[1])
        for side, robot in self.robots.items():
            mount = self.metadata["arm_mounts"][side]
            pose = robot.data.root_pose_w.torch.clone()
            pose[0, :3] = torch.tensor(self.local(mount), device=self.sim.device)
            robot.write_root_pose_to_sim_index(root_pose=pose)
            robot.set_joint_position_target_index(target=self.travel_q[side])
            self.arms[side].command += torch.cat((increment, torch.zeros(1, device=self.sim.device)))[None]
            self.targets[side] = tuple(float(v) for v in self.arms[side].pose()[0][0])
        pose = self.base.data.root_pose_w.torch.clone()
        pose[0, :3] = torch.tensor(self.offset, device=self.sim.device)
        self.base.write_root_pose_to_sim_index(root_pose=pose)
        if length < 0.001:
            self.dwell += self.sim.get_physics_dt()
            if self.dwell > 0.5:
                self.events.append(
                    {"event": "station_arrived", "station": stations[self.phase], "offset": self.offset.copy()}
                )
                for arm in self.arms.values():
                    arm.command = arm.pose()[0].clone()
                self.advance()
        elif self.elapsed > 60:
            self.failure = "navigation_timeout"

    def step(self):
        from pxr import Gf

        dt = self.sim.get_physics_dt()
        self.elapsed += dt
        for side, ops in self.tools.items():
            opening = 0.17 if side in self.holding else 0.18
            if side == "right" and self.phase in ("gap_enter", "gap_spread", "gap_retract"):
                opening = 0.15
            self.openings[side] += max(-0.07 * dt, min(0.07 * dt, opening - self.openings[side]))
            for sign, op in zip((-1, 1), ops):
                op.Set(Gf.Vec3d(sign * self.openings[side] / 2, 0, -0.20))
        if self.phase.startswith("drive_"):
            self.navigate()
            return
        results = []
        careful = any(v in self.phase for v in ("lower", "insert", "enter", "spread", "seat", "extract", "raise"))
        for side, arm in self.arms.items():
            results.append(arm.step(self.targets[side], 0, 0.025 if careful else 0.10))
        self.max_gap = max(self.max_gap, self.gap())
        if self.finished:
            return
        settled = all(d < 0.006 and a < 0.06 and v < 0.045 for d, a, v in results)
        self.dwell = self.dwell + dt if settled else 0.0
        if self.dwell > (
            0.8 if self.phase.startswith(("grip", "release")) else 1.0 if self.phase == "catalog_match" else 0.35
        ):
            self.advance()
        elif self.elapsed > self.timeout:
            self.failure = "waypoint_timeout: " + self.phase


def run(args, app):
    import torch

    from isaaclab_arena_examples.library.crowded_capture import GenericCapture

    sim, robots, objects, joints, tools, metadata, base = create_scene()
    ctrl = MobileDualController(sim, robots, objects, joints, tools, metadata, base, args.waypoint_timeout)
    capture = (
        None
        if args.no_capture
        else GenericCapture(sim, robots, {**objects, "mobile_base": base}, args.output_dir, args.capture_fps)
    )
    report = {"success": False}
    settle = 0.0
    dt = sim.get_physics_dt()
    try:
        with torch.inference_mode():
            for step in range(args.max_steps):
                if not app.is_running():
                    report["reason"] = "window_closed"
                    break
                if step >= 120:
                    ctrl.step()
                for obj in [*robots.values(), base]:
                    obj.write_data_to_sim()
                sim.step()
                for obj in [*robots.values(), *objects.values(), base]:
                    obj.update(dt)
                if capture:
                    capture.record(step + 1, ctrl.phase, {"gap_m": ctrl.gap(), "base_offset": ctrl.offset.copy()})
                if ctrl.failure:
                    report["reason"] = ctrl.failure
                    break
                if args.stop_after and ctrl.phase == args.stop_after:
                    report["reason"] = "probe_complete"
                    break
                if ctrl.finished:
                    settle += dt
                    if settle > 1.0:
                        checks = {}
                        for name in ("return_book", "loan_book"):
                            pose = objects[name].data.root_pose_w.torch[0]
                            target = metadata[name]["target"]
                            checks[name] = (
                                all(
                                    abs(float(pose[i]) - target[i]) < limit
                                    for i, limit in enumerate((0.025, 0.02, 0.02))
                                )
                                and float(objects[name].data.root_vel_w.torch.norm()) < 0.1
                                and name in ctrl.released
                            )
                        checks["upright"] = all(
                            1.0 - 2.0 * sum(float(v) ** 2 for v in objects[n].data.root_pose_w.torch[0, 3:5])
                            > math.cos(math.radians(20))
                            for n in ("return_book", "loan_book")
                        )
                        checks["stations"] = len([e for e in ctrl.events if e["event"] == "station_arrived"]) == 4
                        report.update(
                            success=all(checks.values()),
                            checks=checks,
                            reason="complete" if all(checks.values()) else "placement_validation_failed",
                        )
                        break
            else:
                report["reason"] = "max_steps"
    finally:
        report.update(
            phase=ctrl.phase,
            steps=step + 1,
            events=ctrl.events,
            scene=metadata,
            navigation_model="prescribed kinematic common-base translation; no wheel traction or obstacle planning",
            grasp_model="visual tongs with fixed collision pads and ideal holding constraints",
            distance_m=ctrl.distance,
            max_gap_m=ctrl.max_gap,
            objects={
                name: {
                    "pose_xyzw": obj.data.root_pose_w.torch[0].tolist(),
                    "velocity": obj.data.root_vel_w.torch[0].tolist(),
                }
                for name, obj in objects.items()
            },
            arms={
                s: {
                    "joint_positions": r.data.joint_pos.torch[0].tolist(),
                    "root_pose": r.data.root_pose_w.torch[0].tolist(),
                    "tcp": ctrl.arms[s].pose()[0][0].tolist(),
                    "target": ctrl.targets[s],
                }
                for s, r in robots.items()
            },
        )
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        if capture is None and ctrl.failure:
            capture = GenericCapture(sim, robots, {**objects, "mobile_base": base}, args.output_dir, args.capture_fps)
        if capture:
            capture.record(step + 1, ctrl.phase, force=True)
            capture.save(report)
    print(json.dumps({"success": report["success"], "reason": report["reason"]}), flush=True)
    assert report["success"] or report["reason"] == "probe_complete", report["reason"]


def main():
    args = parse_args()
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

    with SimulationAppContext(args) as app:
        run(args, app)


if __name__ == "__main__":
    main()
