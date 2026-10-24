# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Build two library shelves and separated return and checkout boxes for Galbot S1."""

BOOK_SIZE = (0.16, 0.035, 0.22)
STATIONS = {"boxes": (0.12, -2.31, 0.0), "shelf_a": (0.42, 0.0, 0.0), "shelf_b": (0.42, 1.80, 0.0)}
from isaaclab_arena_examples.library.s1_library_plan import catalog_for

CATALOG = catalog_for(1)


def create_library(
    sim,
    dynamic_neighbors=False,
    blocker_angle=0,
    batch_books=1,
    expanded_room=False,
    dense_shelves=False,
    packed_shelves=False,
):
    """Return dynamic target books and metadata in a collidable library scene.

    Args:
        sim: Simulation context before reset.
        dynamic_neighbors: Give working-shelf neighbors gravity and collision dynamics.
        blocker_angle: Initial blocker roll in degrees for contact trials.
        batch_books: Number of returns and loans, each.
        expanded_room: Stock additional bookcases outside the working aisle.
        dense_shelves: Stock working rows at 41 mm pitch with a shorter blocker relocation.
        packed_shelves: Use uniform 42 mm pitch at both task positions and physical row clearing.

    Returns:
        Target book mapping and scene metadata.
    """
    import math
    from scipy.spatial.transform import Rotation

    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObject, RigidObjectCfg

    books = {}
    catalog = catalog_for(batch_books)
    assert not dense_shelves or batch_books == 1
    if dense_shelves:
        catalog["blocker"]["target"] = (1.35, -0.09, 0.97)

    if packed_shelves:
        assert dense_shelves and abs(blocker_angle) < 1e-9, "Packed rows require upright initial books."
        catalog["blocker"]["target"] = (1.35, -0.042, 0.97)
    clearance_neighbors = []

    friction = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4, restitution=0.0)

    def box(name, size, xyz, color, collision=True, parent="/World/Library", mass=None):
        cfg = sim_utils.CuboidCfg(
            size=size,
            collision_props=(
                sim_utils.CollisionPropertiesCfg(contact_offset=0.0003 if dynamic_neighbors else 0.001, rest_offset=0.0)
                if collision
                else None
            ),
            rigid_props=(
                sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=32, solver_velocity_iteration_count=8)
                if mass
                else None
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=mass) if mass else None,
            activate_contact_sensors=mass is not None,
            physics_material=friction if collision else None,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.75),
        )
        if mass:
            books[name] = RigidObject(
                RigidObjectCfg(
                    prim_path=parent + "/" + name,
                    spawn=cfg,
                    init_state=RigidObjectCfg.InitialStateCfg(pos=xyz),
                )
            )
        else:
            cfg.func(parent + "/" + name, cfg, translation=xyz)

    wood = (0.38, 0.24, 0.13)
    colors = [(0.13, 0.31, 0.51), (0.45, 0.20, 0.17), (0.27, 0.43, 0.24), (0.45, 0.29, 0.48)]
    box("Floor", (5.8, 7.2, 0.10), (0.0, -0.30, -0.09), (0.55, 0.48, 0.38))
    box("RearWall", (0.08, 7.2, 2.6), (2.1, -0.3, 1.3), (0.82, 0.84, 0.79))
    box("SideWall", (3.3, 0.08, 2.6), (0.45, 3.25, 1.3), (0.78, 0.82, 0.83))
    for letter, cy, surface in (("A", 0.0, 0.86), ("B", 1.80, 1.52)):
        box(f"Shelf{letter}Board", (0.40, 1.0, 0.035), (1.36, cy, surface - 0.0175), wood)
        box(f"Shelf{letter}Back", (0.025, 1.0, 2.14), (1.56, cy, 1.03), wood)
        for side in (-1, 1):
            box(f"Shelf{letter}Side{side+1}", (0.4, 0.025, 2.14), (1.36, cy + side * 0.5, 1.03), wood)
        box(f"Shelf{letter}UpperBoard", (0.4, 1.0, 0.03), (1.36, cy, 1.62 if letter == "A" else 0.72), wood)
        box(f"Shelf{letter}Top", (0.4, 1.0, 0.03), (1.36, cy, 2.1), wood)
        for i in range(18 if expanded_room else 11):
            box(
                f"Shelf{letter}UpperBook{i}",
                (0.16, 0.035 if expanded_room else 0.06, 0.24),
                (
                    1.35,
                    cy - (0.425 if expanded_room else 0.4) + i * (0.05 if expanded_room else 0.077),
                    1.755 if letter == "A" else 0.855,
                ),
                colors[i % 4],
            )
        positions = (
            [-0.35, -0.30, -0.25, 0.05, 0.10, 0.15, 0.22, 0.27, 0.32, 0.37]
            if letter == "A"
            else [-0.36, -0.30, -0.14, -0.08, -0.02, 0.04, 0.10, 0.16, 0.22, 0.28, 0.34]
        )
        if dense_shelves:
            # 35 mm books at 41 mm pitch; reserve the moving jaw envelope.
            positions = (
                [
                    -0.385,
                    -0.344,
                    -0.303,
                    -0.262,
                    -0.221,
                    -0.180,
                    0.05,
                    0.091,
                    0.132,
                    0.173,
                    0.214,
                    0.255,
                    0.296,
                    0.337,
                    0.378,
                ]
                if letter == "A"
                else [
                    -0.373,
                    -0.332,
                    -0.14,
                    -0.099,
                    -0.058,
                    -0.017,
                    0.024,
                    0.065,
                    0.106,
                    0.147,
                    0.188,
                    0.229,
                    0.270,
                    0.311,
                    0.352,
                    0.393,
                ]
            )
        if packed_shelves:
            # Both target books belong to the same uniform row as every neighbor.
            # A has a book at zero (the right-hand clearance book); B has the loan at -0.21.
            positions = (
                [round(i * 0.042, 6) for i in range(-10, 11) if i != 0]
                if letter == "A"
                else [round(-0.21 + i * 0.042, 6) for i in range(-5, 16) if i != 0]
            )
        for i, y in enumerate(positions):
            if batch_books == 2 and ((letter == "A" and y == -0.30) or (letter == "B" and y == 0.22)):
                continue
            delta = (
                max(
                    0,
                    (
                        0.035 * abs(math.cos(math.radians(blocker_angle)))
                        + 0.22 * abs(math.sin(math.radians(blocker_angle)))
                    )
                    / 2
                    + 0.0175
                    + 0.006
                    - 0.05,
                )
                if letter == "A" and y > 0
                else 0
            )
            if packed_shelves and letter == "A" and y < 0:
                clearance_neighbors.append(f"Shelf{letter}Neighbor{i}")
            box(
                f"Shelf{letter}Neighbor{i}",
                BOOK_SIZE,
                (1.35, cy + y + delta, surface + 0.11),
                ((0.10, 0.50, 0.25) if letter == "A" and y == 0.05 else colors[i % 4]),
                mass=0.18 if dynamic_neighbors else None,
            )

    for name, cy, color in (
        ("ReturnBox", -2.10, (0.52, 0.28, 0.12)),
        ("CheckoutBox", -1.55, (0.12, 0.38, 0.46)),
    ):
        box(name + "Table", (0.48, 0.42, 0.04), (0.85, cy, 0.84), wood)
        box(name + "Pedestal", (0.20, 0.25, 0.86), (0.85, cy, 0.39), (0.30, 0.33, 0.35))
        for side in (-1, 1):
            box(name + f"Side{side+1}", (0.36, 0.012, 0.06), (0.85, cy + side * 0.15, 0.89), color)
            box(name + f"End{side+1}", (0.012, 0.30, 0.06), (0.85 + side * 0.18, cy, 0.89), color)
    for i in range(3):
        box(f"ReturnStack{i}", (0.18, 0.22, 0.03), (0.85, -2.65, 0.875 + i * 0.031), colors[i], False)
    box("ReturnStackPedestal", (0.20, 0.25, 0.86), (0.85, -2.65, 0.39), (0.30, 0.33, 0.35))
    box("ReturnStackTable", (0.3, 0.35, 0.04), (0.85, -2.65, 0.84), wood)
    room = dict(shelves=2, added_shelves=0, added_static_books=0)
    if expanded_room:
        from isaaclab_arena_examples.library.s1_library_room import add_library_atmosphere

        room = add_library_atmosphere(box)
    for name, entry in catalog.items():
        path = "/World/Book_" + name
        cfg = sim_utils.CuboidCfg(
            size=BOOK_SIZE,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=32, solver_velocity_iteration_count=8
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.0003 if dynamic_neighbors else 0.001, rest_offset=0.0
            ),
            activate_contact_sensors=dynamic_neighbors,
            mass_props=sim_utils.MassPropertiesCfg(mass=0.18),
            physics_material=friction,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.87, 0.84, 0.75)),
        )
        angle = math.radians(blocker_angle) if name == "blocker" else 0
        source = list(entry["source"])
        if angle:
            source[2] = 0.86 + (0.22 * abs(math.cos(angle)) + 0.035 * abs(math.sin(angle))) / 2 + 0.001
        books[name] = RigidObject(
            RigidObjectCfg(
                prim_path=path,
                spawn=cfg,
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=tuple(source), rot=tuple(Rotation.from_euler("x", angle).as_quat())
                ),
            )
        )
        color = {
            "return_book": (0.65, 0.09, 0.07),
            "blocker": (0.50, 0.30, 0.12),
            "loan_book": (0.10, 0.50, 0.25),
            "return_book2": (0.72, 0.36, 0.10),
            "loan_book2": (0.12, 0.28, 0.58),
        }[name]
        for side in (-1, 1):
            box(f"Cover{side+1}", (0.16, 0.002, 0.22), (0, side * 0.0165, 0), color, False, path)
        box("Spine", (0.003, 0.035, 0.22), (-0.0785, 0, 0), color, False, path)
        if name.startswith("loan_book"):
            box("LoanTag", (0.001, 0.027, 0.050), (-0.0804, 0, 0.055), (0.98, 0.98, 0.95), False, path)
    return books, {
        "catalog": catalog,
        "room": room,
        "batch_books": batch_books,
        "stations": STATIONS,
        "book_size": BOOK_SIZE,
        "dense_shelves": dense_shelves,
        "packed_shelves": packed_shelves,
        "clearance_neighbors": clearance_neighbors,
        "clearance_neighbor_motion_limit_m": 0.070,
        "clearance_neighbor_rotation_limit_deg": 8.0,
        "initial_task_pitch_m": 0.042 if packed_shelves else None,
        "target_book_has_unique_white_tag": True,
        "bookends": False,
        "working_row_pitch_m": 0.042 if packed_shelves else 0.041 if dense_shelves else None,
        "working_row_book_gap_m": 0.007 if packed_shelves else 0.006 if dense_shelves else None,
        "slot_clear_width_m": 0.049 if packed_shelves else 0.105 if dense_shelves else 0.165,
        "return_neighbors_y_m": [-0.042, 0.042] if packed_shelves else [-0.09 if dense_shelves else -0.15, 0.05],
        "return_position_initially_occupied": packed_shelves,
        "shelf_neighbors": "dynamic working-shelf books" if dynamic_neighbors else "fixed supporting books",
        "floor_z": -0.04,
    }
