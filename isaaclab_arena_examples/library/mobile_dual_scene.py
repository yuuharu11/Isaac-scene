# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Build separated library stations and one wheeled platform for two arms."""

from __future__ import annotations

import math

SURFACE_Z = 0.75
BOOK_SIZE = (0.035, 0.16, 0.22)
RETURN_SOURCE = (-2.30, -0.98, 0.86)
RETURN_TARGET = (-0.19, 0.52, 0.86)
LOAN_SOURCE = (2.20, 0.52, 0.86)
LOAN_TARGET = (-1.70, -0.98, 0.86)


STATIONS = {
    "boxes": (-2.0, -1.50, 0.0),
    "open_gap": (-0.35, 0.10, 0.0),
    "return_shelf": (0.15, 0.08, 0.0),
    "loan_shelf": (1.65, 0.08, 0.0),
}
ARM_MOUNTS = {"left": (-0.65, -0.10, 0.75), "right": (0.60, -0.20, 0.75)}


def create_mobile_dual_scene(sim):
    """Create furniture and dynamic books and return objects with task metadata.

    Args:
        sim: Initialized Isaac Lab SimulationContext before its first reset.

    Returns:
        Rigid book mapping, JSON-compatible metadata, and kinematic mobile platform.
    """
    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObject, RigidObjectCfg

    material = sim_utils.RigidBodyMaterialCfg(
        static_friction=0.12,
        dynamic_friction=0.08,
        restitution=0.0,
        friction_combine_mode="min",
        restitution_combine_mode="min",
    )

    def box(name, size, position, color, collision=True):
        cfg = sim_utils.CuboidCfg(
            size=size,
            collision_props=(
                sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0.0) if collision else None
            ),
            physics_material=material if collision else None,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.75),
        )
        cfg.func(f"/World/MobileDualLibrary/{name}", cfg, translation=position)

    wood = (0.39, 0.23, 0.12)
    box("LibraryFloor", (8.0, 6.0, 0.10), (0, -0.5, -0.05), (0.55, 0.48, 0.36))
    box("RearWall", (8.0, 0.08, 2.6), (0, 1.65, 1.3), (0.80, 0.82, 0.76))
    for shelf, offset in (("A", 0.0), ("B", 2.0)):

        def shelf_box(name, size, position, color=wood):
            box(f"Shelf{shelf}{name}", size, (position[0] + offset, *position[1:]), color)

        shelf_box("Floor", (0.94, 0.36, 0.035), (0, 0.54, SURFACE_Z - 0.0175))
        shelf_box(
            "BookRearStop", (0.90, 0.01, 0.30 if shelf == "A" else 0.10), (0, 0.611, 0.90 if shelf == "A" else 0.80)
        )
        shelf_box("BookFrontLip", (0.90, 0.01, 0.015), (0, 0.431, 0.7575))
        if shelf == "A":
            # A conventional bookend supports the row after the gripper withdraws.
            shelf_box("Bookend", (0.015, 0.18, 0.25), (0.065, 0.52, 0.875), (0.20, 0.31, 0.34))
        shelf_box("Back", (0.94, 0.025, 2.17), (0, 0.7125, 1.085))
        shelf_box("Upper", (0.94, 0.36, 0.035), (0, 0.54, 2.1675))
        for side in (-1, 1):
            shelf_box(f"Side{side + 1}", (0.025, 0.36, 2.17), (side * 0.4625, 0.54, 1.085))
            shelf_box(f"Leg{side + 1}", (0.045, 0.30, 0.715), (side * 0.43, 0.54, 0.3575))
        # Fill a separate upper row without obstructing the working volume.
        shelf_box("BackgroundBoard", (0.90, 0.32, 0.025), (0, 0.54, 2.205))
        for index in range(10):
            shelf_box(
                f"BackgroundBook{index}",
                (0.065, 0.16, 0.23),
                (-0.38 + index * 0.083, 0.54, 2.3325),
                ((0.20, 0.34, 0.50), (0.50, 0.23, 0.16), (0.30, 0.43, 0.27))[index % 3],
            )
        box(
            f"Shelf{shelf}Marker",
            (0.18, 0.008, 0.06),
            (offset, 0.352, 0.67),
            (0.65, 0.12, 0.09) if shelf == "A" else (0.12, 0.52, 0.26),
            False,
        )

    def open_box(name, x, y, color):
        wall = 0.012
        inner_width, inner_depth = 0.16, 0.24
        box(f"{name}Floor", (inner_width + 2 * wall, inner_depth + 2 * wall, 0.035), (x, y, SURFACE_Z - 0.0175), color)
        for side in (-1, 1):
            box(
                f"{name}Side{side + 1}",
                (wall, inner_depth + 2 * wall, 0.07),
                (x + side * (inner_width + wall) / 2, y, SURFACE_Z + 0.035),
                color,
            )
            box(
                f"{name}End{side + 1}",
                (inner_width, wall, 0.07),
                (x, y + side * (inner_depth + wall) / 2, SURFACE_Z + 0.035),
                color,
            )
        box(f"{name}Pedestal", (0.13, 0.19, 0.715), (x, y, 0.3575), (0.34, 0.38, 0.37))

    open_box("ReturnBox", *RETURN_SOURCE[:2], (0.56, 0.31, 0.12))
    open_box("LoanBox", *LOAN_TARGET[:2], (0.16, 0.36, 0.44))
    colors = ((0.18, 0.32, 0.52), (0.54, 0.37, 0.13), (0.40, 0.22, 0.45), (0.26, 0.44, 0.40))
    objects, descriptions = {}, {}

    def add_book(name, position, color, catalog_id, angle=0.0):
        cfg = sim_utils.CuboidCfg(
            size=BOOK_SIZE,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=32,
                solver_velocity_iteration_count=8,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0.0),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.18),
            physics_material=material,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.88, 0.85, 0.74), roughness=0.75),
        )
        path = f"/World/Book_{name}"
        quaternion = (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))
        objects[name] = RigidObject(
            RigidObjectCfg(
                prim_path=path,
                spawn=cfg,
                init_state=RigidObjectCfg.InitialStateCfg(pos=position, rot=quaternion),
            )
        )

        def visual(part_name, size, offset, tint):
            part = sim_utils.CuboidCfg(
                size=size,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=tint, roughness=0.7),
            )
            part.func(f"{path}/{part_name}", part, translation=offset)

        for side in (-1, 1):
            visual(f"Cover{side + 1}", (0.002, 0.16, 0.22), (side * 0.0165, 0, 0), color)
        visual("Spine", (0.035, 0.003, 0.22), (0, -0.0785, 0), color)
        visual("CatalogLabel", (0.025, 0.0008, 0.035), (0, -0.0801, -0.045), (0.95, 0.93, 0.84))
        for line in range(5):
            visual(f"PageEdge{line}", (0.029, 0.0008, 0.0006), (0, 0.0801, -0.072 + line * 0.036), (0.62, 0.59, 0.51))
        descriptions[name] = {
            "catalog_id": catalog_id,
            "source": list(position),
            "quaternion_xyzw": list(quaternion),
            "size": list(BOOK_SIZE),
            "mass": 0.18,
            "dynamic": True,
        }

    add_book("return_book", RETURN_SOURCE, (0.65, 0.12, 0.09), "LIB-001")
    add_book("loan_book", LOAN_SOURCE, (0.12, 0.52, 0.26), "LIB-003")
    return_left, return_right, loan_neighbors = [], [], []
    index = 0
    for group, positions in (
        (return_left, (-0.40, -0.36, -0.32, -0.28, -0.24)),
        (return_right, (-0.18, -0.14, -0.10, -0.06)),
    ):
        for x in positions:
            name = f"neighbor_{index:02d}"
            add_book(name, (x, 0.52, 0.86), colors[index % len(colors)], f"NBR-{index:03d}")
            group.append(name)
            index += 1
    angle = math.radians(15)
    tilted_height = SURFACE_Z + 0.11 * math.cos(angle) + 0.0175 * math.sin(angle)
    tilted_name = f"neighbor_{index:02d}"
    add_book(tilted_name, (2.135, 0.52, tilted_height), colors[index % len(colors)], f"NBR-{index:03d}", angle)
    loan_neighbors.append(tilted_name)
    index += 1
    for x in (2.24, 2.28, 2.32, 2.36, 2.40):
        name = f"neighbor_{index:02d}"
        add_book(name, (x, 0.52, 0.86), colors[index % len(colors)], f"NBR-{index:03d}")
        loan_neighbors.append(name)
        index += 1
    metadata = {
        "stations": {name: list(position) for name, position in STATIONS.items()},
        "arm_mounts": {name: list(position) for name, position in ARM_MOUNTS.items()},
        "surface_z": SURFACE_Z,
        "book_size": list(BOOK_SIZE),
        "return_book": {
            "catalog_id": "LIB-001",
            "source": list(RETURN_SOURCE),
            "target": list(RETURN_TARGET),
            "shelf_slot": "A-01",
        },
        "loan_book": {
            "catalog_id": "LIB-003",
            "source": list(LOAN_SOURCE),
            "target": list(LOAN_TARGET),
            "shelf_slot": "B-03",
        },
        "return_gap_initial": 0.025,
        "return_gap_bounds_initial": [-0.2225, -0.1975],
        "return_left_neighbors": return_left,
        "return_right_neighbors": return_right,
        "loan_neighbors": loan_neighbors,
        "leaning_neighbor": tilted_name,
        "leaning_angle_degrees": 15.0,
        "helper_push_distance": 0.04,
        "objects": descriptions,
    }
    return objects, metadata, create_dual_mobile_base(sim)


def create_dual_mobile_base(sim):
    """Create a shared kinematic platform; its root pose is at floor height.

    Args:
        sim: Initialized SimulationContext before its first reset.

    Returns:
        RigidObject whose root can be moved together with both articulation roots.
    """
    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from pxr import Gf, UsdGeom, UsdPhysics

    path = "/World/MobileDualBase"
    root = UsdGeom.Xform.Define(sim.stage, path)
    root.AddTranslateOp().Set(Gf.Vec3d(*STATIONS["boxes"]))
    body = UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    body.CreateRigidBodyEnabledAttr(True)
    body.CreateKinematicEnabledAttr(True)

    def part(name, size, position, color):
        cfg = sim_utils.CuboidCfg(
            size=size,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.6),
        )
        cfg.func(f"{path}/{name}", cfg, translation=position)

    part("Chassis", (1.65, 0.72, 0.18), (0, -0.20, 0.22), (0.14, 0.23, 0.31))
    part("FrontPanel", (0.65, 0.012, 0.06), (0, 0.166, 0.22), (0.22, 0.67, 0.70))
    for side, mount in ARM_MOUNTS.items():
        x, y, z = mount
        part(f"{side}Column", (0.24, 0.25, z - 0.36), (x, y, (z + 0.26) / 2), (0.55, 0.59, 0.61))
        part(f"{side}Mount", (0.32, 0.30, 0.05), (x, y, z - 0.025), (0.27, 0.31, 0.35))
    for xside in (-1, 1):
        for yside in (-1, 1):
            cfg = sim_utils.CylinderCfg(
                radius=0.12,
                height=0.065,
                axis="X",
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.055, 0.065, 0.075), roughness=0.9),
            )
            cfg.func(
                f"{path}/Wheel{xside + 1}_{yside + 1}", cfg, translation=(xside * 0.82, -0.20 + yside * 0.26, 0.125)
            )
    return RigidObject(
        RigidObjectCfg(
            prim_path=path,
            spawn=None,
            init_state=RigidObjectCfg.InitialStateCfg(pos=STATIONS["boxes"]),
        )
    )
