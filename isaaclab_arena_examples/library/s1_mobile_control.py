# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Drive real S1 wheel, arm and jaw actuators with measured physical feedback."""

import json
import math


def grounded_base_height(model, floor_z):
    """Compute initial chassis height from official wheel centers and native cylinder radius.

    Args:
        model: S1 kinematics containing the authored URDF joint origins.
        floor_z: Measured library floor surface height.

    Returns:
        Chassis height placing all four wheel surfaces on the floor before stepping.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    heights = []
    for index in range(1, 5):
        chain, link = [], f"wheel_driving_link{index}"
        while link in model.joints:
            joint = model.joints[link]
            origin = joint.find("origin")
            transform = np.eye(4)
            if origin is not None:
                transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                transform[:3, :3] = Rotation.from_euler(
                    "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                ).as_matrix()
            chain.append(transform)
            link = joint.find("parent").get("link")
        world = np.eye(4)
        for transform in reversed(chain):
            world = world @ transform
        heights.append(float(floor_z + 0.08272846 - world[2, 3]))
    assert np.ptp(heights) < 1e-8, "Initial wheels must share a support plane."
    return heights[0]


def round_wheel_colliders(root):
    """Use round wheel contact with the official dimensions, inertias and display meshes.

    Args:
        root: Spawned robot prim whose four wheel collision meshes are replaced.
    """
    import numpy as np

    import carb
    from omni.physx.bindings._physx import SETTING_COLLISION_APPROXIMATE_CYLINDERS
    from pxr import Usd, UsdGeom, UsdPhysics

    carb.settings.get_settings().set_bool(SETTING_COLLISION_APPROXIMATE_CYLINDERS, False)
    for i in range(1, 5):
        stage = root.GetStage()
        link_path = str(root.GetPath()) + f"/wheel_driving_link{i}"
        collisions = stage.GetPrimAtPath(link_path + "/collisions")
        for prim in list(Usd.PrimRange(collisions)):
            if prim.IsInstance():
                prim.SetInstanceable(False)
        original = [p for p in Usd.PrimRange(collisions) if p.HasAPI(UsdPhysics.CollisionAPI)]
        assert len(original) == 1, "Expected one official wheel collision mesh."
        old = original[0]
        points = np.asarray(UsdGeom.Mesh(old).GetPointsAttr().Get(), dtype=float)
        radius = float(np.linalg.norm(points[:, :2], axis=1).max())
        lower, upper = float(points[:, 2].min()), float(points[:, 2].max())
        assert abs(radius - 0.08272846) < 1e-6 and abs(lower + upper) < 1e-6
        UsdPhysics.CollisionAPI(old).GetCollisionEnabledAttr().Set(False)
        cylinder = UsdGeom.Cylinder.Define(stage, link_path + "/RollingContact")
        cylinder.CreateRadiusAttr(radius)
        cylinder.CreateHeightAttr(upper - lower)
        cylinder.CreateAxisAttr("Z")
        cylinder.CreatePurposeAttr("guide")
        cylinder.CreateVisibilityAttr("invisible")
        UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
        for rel in old.GetRelationships():
            if rel.GetName().startswith("material:binding"):
                cylinder.GetPrim().CreateRelationship(rel.GetName()).SetTargets(rel.GetTargets())
        print(f"[wheel-geometry] wheel={i} cylinder radius={radius:.9f} width={upper-lower:.9f}", flush=True)


def measure_round_wheels(sim, robot, directory):
    """Measure actual physics-scene wheel radii with radial rays and save inertial properties.

    Args:
        sim: Initialized physics context with scene queries enabled.
        robot: Articulation with current measured body poses.
        directory: Capture directory receiving wheel_geometry.json.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    import carb
    from omni.physx import get_physx_scene_query_interface, get_physxunittests_interface
    from omni.physx.bindings._physx import SETTING_COLLISION_APPROXIMATE_CYLINDERS
    from pxr import UsdGeom, UsdPhysics

    assert not carb.settings.get_settings().get(SETTING_COLLISION_APPROXIMATE_CYLINDERS)
    records = []
    for i in range(1, 5):
        name = f"wheel_driving_link{i}"
        link = "/World/Galbot/" + name
        path = link + "/RollingContact"
        pose = robot.data.body_link_pose_w.torch[0, robot.body_names.index(name)].numpy()
        rotation = Rotation.from_quat(pose[3:])
        radii = []
        for angle in np.arange(128) * (2 * math.pi / 128):
            outward = rotation.apply([math.cos(angle), math.sin(angle), 0])
            hits = []

            def receive(hit):
                if hit.collision == path:
                    hits.append(0.12 - hit.distance)
                return True

            get_physx_scene_query_interface().raycast_all(
                (pose[:3] + outward * 0.12).tolist(), (-outward).tolist(), 0.1, receive
            )
            assert len(hits) == 1, "Could not isolate a wheel contact surface."
            radii.append(hits[0])
        cylinder = UsdGeom.Cylinder(sim.stage.GetPrimAtPath(path))
        radius = cylinder.GetRadiusAttr().Get()
        mass = UsdPhysics.MassAPI(sim.stage.GetPrimAtPath(link))
        record = dict(
            wheel=i,
            radial_samples=len(radii),
            minimum_radius_m=min(radii),
            maximum_radius_m=max(radii),
            radial_variation_m=float(np.ptp(radii)),
            radius_m=radius,
            width_m=cylinder.GetHeightAttr().Get(),
            mass_kg=mass.GetMassAttr().Get(),
            diagonal_inertia_kg_m2=list(mass.GetDiagonalInertiaAttr().Get()),
        )
        records.append(record)
    result = dict(
        wheels=records,
        statistics=get_physxunittests_interface().get_physics_stats(),
        convex_cylinder_approximation=False,
    )
    (directory / "wheel_geometry.json").write_text(json.dumps(result, indent=2) + "\n")
    assert all(
        r["radial_variation_m"] < 1e-5 and abs(r["radius_m"] - r["minimum_radius_m"]) < 0.00011 for r in records
    ), "Wheel contact queries did not confirm round geometry."


def spawn_mobile_robot(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Spawn the official S1 with both stock grippers removed for spatula mounts."""
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from pxr import PhysxSchema, Usd, UsdPhysics

    root = spawn_from_usd(prim_path, cfg, translation=translation, orientation=orientation, **kwargs)
    for prim in list(Usd.PrimRange(root)):
        if (
            prim.IsValid()
            and prim.GetName().startswith(("left_", "right_"))
            and any(
                key in prim.GetName()
                for key in (
                    "active_link",
                    "active_joint",
                    "passive_link",
                    "passive_joint",
                    "gripper_base_link",
                    "fixed_contact_link",
                    "active_contact_link",
                )
            )
        ):
            prim.SetActive(False)
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0)
    if getattr(cfg, "library_cylinder_wheels", False):
        round_wheel_colliders(root)
    if not cfg.articulation_props.fix_root_link:
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                    prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        base = root.GetStage().GetPrimAtPath(prim_path + "/base_link")
        UsdPhysics.ArticulationRootAPI.Apply(base)
        api = PhysxSchema.PhysxArticulationAPI.Apply(base)
        api.CreateEnabledSelfCollisionsAttr(False)
        api.CreateSolverPositionIterationCountAttr(32)
        api.CreateSolverVelocityIterationCountAttr(8)
    return root


def robot_config(position, cylinder_wheels=False):
    """Return a floating S1 with gravity and force-limited wheel actuators."""
    from isaaclab.actuators import ImplicitActuatorCfg

    from isaaclab_arena_examples.library.galbot_s1_robot import make_galbot_cfg

    cfg = make_galbot_cfg(position=tuple(position))
    cfg.spawn.func = spawn_mobile_robot
    cfg.spawn.library_cylinder_wheels = cylinder_wheels
    cfg.spawn.rigid_props.disable_gravity = False
    cfg.spawn.activate_contact_sensors = True
    cfg.spawn.articulation_props.fix_root_link = False
    cfg.actuators.pop("hands")
    cfg.actuators.pop("wheels")
    cfg.actuators["steering"] = ImplicitActuatorCfg(
        joint_names_expr=["wheel_steering_.*"],
        stiffness=1000,
        damping=50,
        effort_limit_sim=100,
        velocity_limit_sim=1.5,
    )
    cfg.actuators["driving"] = ImplicitActuatorCfg(
        joint_names_expr=["wheel_driving_.*"],
        stiffness=0,
        damping=50,
        effort_limit_sim=20,
        velocity_limit_sim=1.5,
    )
    return cfg


class WheelController:
    """Command four steering and driving joints from measured chassis pose."""

    def __init__(self, robot, max_speed=0.09, max_acceleration=0.06):
        import numpy as np

        self.robot = robot
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.steer = [robot.joint_names.index(f"wheel_steering_joint{i}") for i in range(1, 5)]
        self.drive = [robot.joint_names.index(f"wheel_driving_joint{i}") for i in range(1, 5)]
        from isaaclab_arena_examples.library.s1_library_ik import S1Kinematics

        model = S1Kinematics()
        self.offsets = np.array([
            np.fromstring(model.by_name[f"wheel_steering_joint{i}"].find("origin").get("xyz"), sep=" ")[:2]
            for i in range(1, 5)
        ])
        self.velocity = np.zeros(2)
        self.integral = np.zeros(4)

    def command(self, goal, q, dt):
        """Set finite wheel motor targets without overwriting chassis pose or velocity."""
        import numpy as np
        import torch
        from scipy.spatial.transform import Rotation

        robot = self.robot
        pose = robot.data.root_link_pose_w.torch[0].numpy()
        yaw = Rotation.from_quat(pose[3:]).as_euler("xyz")[2]
        world = 0.8 * (np.asarray(goal)[:2] - pose[:2])
        if np.linalg.norm(np.asarray(goal)[:2] - pose[:2]) < 0.003:
            world[:] = 0
        world *= min(1, self.max_speed / max(np.linalg.norm(world), 1e-12))
        change = world - self.velocity
        self.velocity += change * min(1, self.max_acceleration * dt / max(np.linalg.norm(change), 1e-12))
        c, s = math.cos(yaw), math.sin(yaw)
        body = np.array([[c, s], [-s, c]]) @ self.velocity
        omega = float(np.clip(-2 * yaw - 0.5 * robot.data.root_com_vel_w.torch[0, 5].item(), -0.15, 0.15))
        if abs(yaw) < 0.005 and abs(robot.data.root_com_vel_w.torch[0, 5].item()) < 0.01:
            omega = 0
        target_v = torch.zeros_like(q)
        for index, (x, y) in enumerate(self.offsets):
            v = body + omega * np.array([-y, x])
            speed = float(np.linalg.norm(v))
            actual = robot.data.joint_pos.torch[0, self.steer[index]].item()
            angle = math.atan2(v[1], v[0]) if speed > 0.003 else float(q[0, self.steer[index]])
            choices = [
                (angle + turn * math.pi, (-1) ** turn) for turn in (-1, 0, 1) if abs(angle + turn * math.pi) < 2.90
            ]
            angle, direction = min(choices, key=lambda value: abs(value[0] - actual))
            speed *= direction
            q[0, self.steer[index]] = angle
            desired = speed / 0.08272846 * max(0, math.cos(angle - actual)) ** 2
            measured = robot.data.joint_vel.torch[0, self.drive[index]].item()
            self.integral[index] = np.clip(self.integral[index] + 1.5 * dt * (desired - measured), -0.5, 0.5)
            if abs(desired) < 0.001:
                self.integral[index] *= math.exp(-4 * dt)
            target_v[0, self.drive[index]] = float(np.clip(desired + self.integral[index], -1.5, 1.5))
        robot.set_joint_position_target_index(target=q)
        robot.set_joint_velocity_target_index(target=target_v)
        robot.write_data_to_sim()


def control_tools(robot, tools, q, ik, goal_lift, step, dt, period=0.05):
    """Drive both arm chains and jaws using measured floating-base and tool pose feedback."""
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation

    stride = max(1, round(period / dt))
    lift_id = robot.joint_names.index("torso_lift_joint1")
    q[0, lift_id] = goal_lift
    if step % stride == 0:
        base = robot.data.root_link_pose_w.torch[0].numpy()
        base_rotation = Rotation.from_quat(base[3:])
        actual_lift = robot.data.joint_pos.torch[0, lift_id].item()
        for side, t in tools.items():
            actual = t["tool"].data.root_link_pose_w.torch[0].numpy()
            t["correction"] = np.clip(t["correction"] + 2 * stride * dt * (t["goal"] - actual[:3]), -0.03, 0.03)
            error = (t["rotation"] * Rotation.from_quat(actual[3:]).inv()).as_rotvec()
            t["angular_correction"] = np.clip(t["angular_correction"] + 2 * stride * dt * error, -0.08, 0.08)
            rotation = Rotation.from_rotvec(t["angular_correction"]) * t["rotation"]
            wrist = t["goal"] + t["correction"] - rotation.apply([0.15, 0, 0])
            ids = [robot.joint_names.index(f"{side}_arm_joint{i}") for i in range(1, 8)]
            seed = q[0, ids].numpy()
            angles, residual = ik.solve(
                side,
                base_rotation.inv().apply(wrist - base[:3]),
                actual_lift,
                seed,
                rotation=(base_rotation.inv() * rotation * Rotation.from_euler("z", math.pi)).as_matrix(),
            )
            t["arm_joint_ids"] = ids
            actual_q = robot.data.joint_pos.torch[0, ids].numpy()
            limits = np.array([
                [float(ik.by_name[f"{side}_arm_joint{i}"].find("limit").get(key)) for key in ("lower", "upper")]
                for i in range(1, 8)
            ])
            margin = float(np.min(np.minimum(actual_q - limits[:, 0], limits[:, 1] - actual_q)))
            diagnostic = t.setdefault("control_diagnostics", {}).setdefault(
                t.get("phase", "initial"),
                dict(
                    max_target_step_rad=0.0,
                    max_ik_error_m=0.0,
                    min_joint_limit_margin_rad=margin,
                    max_measured_joint_speed_rad_s=0.0,
                ),
            )
            diagnostic["max_target_step_rad"] = max(
                diagnostic["max_target_step_rad"], float(np.max(np.abs(angles - seed)))
            )
            diagnostic["max_ik_error_m"] = max(diagnostic["max_ik_error_m"], float(residual))
            diagnostic["min_joint_limit_margin_rad"] = min(diagnostic["min_joint_limit_margin_rad"], margin)
            diagnostic["max_measured_joint_speed_rad_s"] = max(
                diagnostic["max_measured_joint_speed_rad_s"],
                float(robot.data.joint_vel.torch[0, ids].abs().max().item()),
            )
            q[0, ids] = torch.tensor(angles, dtype=q.dtype)
    for t in tools.values():
        tool = t["tool"]
        tq = tool.data.joint_pos.torch.clone()
        ids = [tool.joint_names.index("jaw_left"), tool.joint_names.index("jaw_right")]
        tq[0, ids] = torch.tensor([-t["opening"], t["opening"]], dtype=tq.dtype)
        tool.set_joint_position_target_index(target=tq)
        tool.write_data_to_sim()


def prepare_full_phase(robot, tools, books, phase, lift):
    """Set measured start poses, grasp offsets and goals for one full-library phase."""
    import numpy as np
    from scipy.spatial.transform import Rotation, Slerp

    from isaaclab_arena_examples.library.dual_spatula_probe import pose

    moving = phase["base"] is not None
    base_pose = robot.data.root_link_pose_w.torch[0].numpy()
    base_rotation = Rotation.from_quat(base_pose[3:])
    for side, t in tools.items():
        t.pop("settling_previous_pose", None)
        t["start"] = t["tool"].data.root_link_pose_w.torch[0, :3].numpy().copy()
        t["start_rotation"] = Rotation.from_quat(t["tool"].data.root_link_pose_w.torch[0, 3:].numpy())
        if moving:
            t["relative"] = base_rotation.inv().apply(t["start"] - base_pose[:3])
            t["relative_rotation"] = base_rotation.inv() * t["start_rotation"]
        if phase["lift"] is not None:
            t["goal"][2] += phase["lift"] - lift
    side = phase["side"]
    action = phase["action"]
    book_name = phase["book"]
    if side:
        t = tools[side]
        if book_name:
            t["active"] = book_name
        if action == "align":
            t["rotation"] = Rotation.from_quat(pose(books[book_name])[3:])
        if action == "upright":
            t["rotation"] = Rotation.identity()
        if phase["xyz"] is not None:
            goal_book = np.array(phase["xyz"])
            if action == "align":
                goal_book[1:] = pose(books[book_name])[1:3]
            offset = np.array([0.24, 0, 0])
            if t["held"] == book_name and book_name:
                actual = t["tool"].data.root_link_pose_w.torch[0].numpy()
                offset = Rotation.from_quat(actual[3:]).inv().apply(pose(books[book_name])[:3] - actual[:3])
            t["goal"] = goal_book - t["rotation"].apply(offset)
        if action == "close":
            t["opening"] = 0.015
        if action == "open":
            t["opening"] = 0.025
    for t in tools.values():
        t["end"] = t["goal"].copy()
        t["slerp"] = Slerp([0, 1], Rotation.concatenate([t["start_rotation"], t["rotation"]]))


def validate_transport_clearance(books, tools, phase):
    """Require every carried book to clear the shelf before backing away or lowering.

    Args:
        books: Measured target rigid objects.
        tools: Side-keyed articulation and grasp states.
        phase: Upcoming phase name.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    from isaaclab_arena_examples.library.dual_spatula_probe import pose

    if phase not in ("exit_shelf_b", "lower_for_travel"):
        return
    limit = 1.25 if phase == "exit_shelf_b" else 1.15
    for tool in tools.values():
        if tool["held"]:
            measured = pose(books[tool["held"]])
            extent = np.abs(Rotation.from_quat(measured[3:]).as_matrix()[0]) @ [0.08, 0.0175, 0.11]
            assert measured[0] + extent < limit, "Held book is not fully clear of the shelf."
        measured = pose(tool["tool"])
        rotation = Rotation.from_quat(measured[3:]).as_matrix()
        tip = measured[0] + rotation[0, 0] * 0.20 + np.abs(rotation[0]) @ [0.06, 0.0615, 0.04]
        assert tip < limit, "Withdraw both tools before shelf departure or lowering."


def jaw_motion_settled(state, book, action, target_index, dt):
    """Check unfiltered physical pose differences, jaw contact and tool position.

    Args:
        state: Mutable tool state retaining the preceding physical pose.
        book: Target rigid object with current measured pose.
        action: Close or open action.
        target_index: Target column in both contact sensors.
        dt: Physical timestep between consecutive samples.

    Returns:
        Whether contact, pose rate, tool position and opening conditions hold.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    forces = [
        float(np.linalg.norm(sensor.data.force_matrix_w.torch[0, 0, target_index].numpy()))
        for sensor in state["sensors"]
    ]
    measured = book.data.root_link_pose_w.torch[0].numpy().copy()
    previous = state.get("settling_previous_pose")
    state["settling_previous_pose"] = measured
    if previous is None:
        return False
    linear_speed = np.linalg.norm(measured[:3] - previous[:3]) / dt
    angular_speed = (Rotation.from_quat(measured[3:]) * Rotation.from_quat(previous[3:]).inv()).magnitude() / dt
    actual = state["tool"].data.root_link_pose_w.torch[0, :3].numpy()
    stable = linear_speed < 0.01 and angular_speed < 0.05
    at_goal = np.linalg.norm(actual - state["end"]) < 0.003
    if action == "close":
        return bool(min(forces) > 0.05 and stable and at_goal)
    ids = [state["tool"].joint_names.index(name) for name in ("jaw_left", "jaw_right")]
    joints = state["tool"].data.joint_pos.torch[0, ids].numpy()
    return bool(max(forces) < 0.05 and stable and at_goal and min(abs(joints)) > 0.024)


def record_jaw_settling(phase, elapsed, quiet, closing_duration, events, enabled):
    """Record and require sustained jaw settling when adaptive waits are enabled.

    Args:
        phase: Completed phase metadata.
        elapsed: Measured phase time in seconds.
        quiet: Consecutive stable time in seconds.
        closing_duration: Commanded closing ramp duration in seconds.
        events: Destination list for successful and failed checks.
        enabled: Whether adaptive waits are active.
    """
    action = phase["action"]
    if not enabled or action not in ("close", "open"):
        return
    settled = quiet >= 0.5 and (action != "close" or elapsed >= closing_duration)
    events.append(dict(phase=phase["name"], elapsed_s=elapsed, quiet_s=quiet, settled=settled))
    assert settled, f"Jaw motion did not settle: {phase['name']}, quiet={quiet:.4f}s"


def set_grasp_force_limit(tools, phase, default_force, loan_force):
    """Set the physical jaw drive limit before closing, retaining it throughout each hold.

    Args:
        tools: Side-keyed tool states with physical articulations.
        phase: Upcoming phase metadata.
        default_force: Default force limit per jaw in newtons.
        loan_force: Optional lower per-jaw limit for borrowed books.
    """
    import numpy as np

    if phase["action"] != "close":
        return
    force = loan_force if phase["book"].startswith("loan_book") and loan_force is not None else default_force
    state = tools[phase["side"]]
    tool = state["tool"]
    tool.write_joint_effort_limit_to_sim_index(limits=float(force))
    actual = tool.root_view.get_dof_max_forces().numpy()[0]
    assert np.allclose(actual, force), "Physical jaw force limit did not match the requested value."
    state.setdefault("force_limit_events", []).append(dict(phase=phase["name"], limit_N=actual.tolist()))


def validate_release_support(books, catalog, phase):
    """Require the measured book bottom to be seated within 5 mm before opening.

    Args:
        books: Target rigid objects with measured poses.
        catalog: Target metadata defining destination support heights.
        phase: Upcoming phase metadata.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    if phase["action"] != "open":
        return
    measured = books[phase["book"]].data.root_link_pose_w.torch[0].numpy()
    bottom = measured[2] - np.abs(Rotation.from_quat(measured[3:]).as_matrix()[2]) @ [0.08, 0.0175, 0.11]
    error = abs(bottom - catalog[phase["book"]]["surface_z"])
    assert error < 0.005, f"Book not seated before release: {phase['name']}, bottom error={error:.6f}m"
