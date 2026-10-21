# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Build two library shelves and separated return and checkout boxes for Galbot S1."""

BOOK_SIZE = (0.16, 0.035, 0.22)
STATIONS = {"boxes": (0.12, -2.31, 0.0), "shelf_a": (0.42, 0.0, 0.0), "shelf_b": (0.42, 1.80, 0.0)}
CATALOG = {
    "return_book": {"id": "LIB-001", "source": (0.85, -2.10, 0.97), "target": (1.35, 0.0, 0.97), "slot": "A-01"},
    "blocker": {"id": "NBR-001", "source": (1.35, 0.0, 0.97), "target": (1.35, -0.15, 0.97), "slot": "A-02"},
    "loan_book": {"id": "LIB-003", "source": (1.35, 1.59, 1.63), "target": (0.85, -1.55, 0.97), "slot": "B-03"},
}


def create_library(sim):
    """Return dynamic target books and metadata in a collidable library scene.

    Args:
        sim: Simulation context before reset.

    Returns:
        Target book mapping and scene metadata.
    """
    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObject, RigidObjectCfg

    friction = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4, restitution=0.0)

    def box(name, size, xyz, color, collision=True, parent="/World/Library"):
        cfg = sim_utils.CuboidCfg(
            size=size,
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0.0) if collision else None,
            physics_material=friction if collision else None,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.75),
        )
        cfg.func(parent + "/" + name, cfg, translation=xyz)

    wood = (0.38, 0.24, 0.13)
    colors = [(0.13, 0.31, 0.51), (0.45, 0.20, 0.17), (0.27, 0.43, 0.24), (0.45, 0.29, 0.48)]
    box("Floor", (5.8, 7.2, 0.10), (0.0, -0.30, -0.09), (0.55, 0.48, 0.38))
    box("RearWall", (0.08, 7.2, 2.6), (2.1, -0.3, 1.3), (0.82, 0.84, 0.79))
    box("SideWall", (3.3, 0.08, 2.6), (0.45, 3.25, 1.3), (0.78, 0.82, 0.83))
    for letter, cy, surface in (("A", 0.0, 0.86), ("B", 1.80, 1.52)):
        box(f"Shelf{letter}Board", (0.40, 1.0, 0.035), (1.36, cy, surface - 0.0175), wood)
        box(f"Shelf{letter}Back", (0.025, 1.0, 2.1), (1.56, cy, 1.05), wood)
        for side in (-1, 1):
            box(f"Shelf{letter}Side{side+1}", (0.4, 0.025, 2.1), (1.36, cy + side * 0.5, 1.05), wood)
            box(f"Shelf{letter}Bookend{side+1}", (0.19, 0.012, 0.24), (1.35, cy + side * 0.42, surface + 0.12), (0.22, 0.28, 0.30))
        box(f"Shelf{letter}UpperBoard", (0.4, 1.0, 0.03), (1.36, cy, 1.62 if letter == "A" else 0.72), wood)
        box(f"Shelf{letter}Top", (0.4, 1.0, 0.03), (1.36, cy, 2.1), wood)
        box(f"Shelf{letter}Marker", (0.008, 0.20, 0.06), (1.155, cy, surface - 0.09), colors[0 if letter == 'A' else 2], False)
        for i in range(11):
            box(f"Shelf{letter}UpperBook{i}", (0.16, 0.06, 0.24), (1.35, cy - 0.4 + i * 0.077, 1.755 if letter == "A" else 0.855), colors[i % 4])
        positions = [-0.35, -0.30, -0.25, 0.05, 0.10, 0.15, 0.22, 0.27, 0.32, 0.37] if letter == "A" else [-0.36, -0.30, -0.14, -0.08, -0.02, 0.04, 0.10, 0.16, 0.22, 0.28, 0.34]
        for i, y in enumerate(positions):
            box(f"Shelf{letter}Neighbor{i}", BOOK_SIZE, (1.35, cy + y, surface + 0.11), ((0.10, 0.50, 0.25) if letter == "A" and y == 0.05 else colors[i % 4]))

    for name, cy, color in (("ReturnBox", -2.10, (0.52, 0.28, 0.12)), ("CheckoutBox", -1.55, (0.12, 0.38, 0.46))):
        box(name + "Table", (0.48, 0.42, 0.04), (0.85, cy, 0.84), wood)
        box(name + "Pedestal", (0.20, 0.25, 0.8), (0.85, cy, 0.4), (0.30, 0.33, 0.35))
        for side in (-1, 1):
            box(name + f"Side{side+1}", (0.36, 0.012, 0.06), (0.85, cy + side * 0.15, 0.89), color)
            box(name + f"End{side+1}", (0.012, 0.30, 0.06), (0.85 + side * 0.18, cy, 0.89), color)
    for i in range(3):
        box(f"ReturnStack{i}", (0.18, 0.22, 0.03), (0.85, -2.65, 0.875 + i * 0.031), colors[i], False)
    box("ReturnStackTable", (0.3, 0.35, 0.04), (0.85, -2.65, 0.84), wood)
    books = {}
    for name, entry in CATALOG.items():
        path = "/World/Book_" + name
        cfg = sim_utils.CuboidCfg(
            size=BOOK_SIZE,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=32, solver_velocity_iteration_count=8),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0.0),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.18), physics_material=friction,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.87, 0.84, 0.75)),
        )
        books[name] = RigidObject(RigidObjectCfg(prim_path=path, spawn=cfg,
            init_state=RigidObjectCfg.InitialStateCfg(pos=entry["source"])))
        color = {"return_book": (0.65, 0.09, 0.07), "blocker": (0.50, 0.30, 0.12), "loan_book": (0.10, 0.50, 0.25)}[name]
        for side in (-1, 1):
            box(f"Cover{side+1}", (0.16, 0.002, 0.22), (0, side * 0.0165, 0), color, False, path)
        box("Spine", (0.003, 0.035, 0.22), (-0.0785, 0, 0), color, False, path)
        box("Label", (0.0008, 0.027, 0.035), (-0.0802, 0, -0.045), (0.94, 0.93, 0.85), False, path)
    return books, {"catalog": CATALOG, "stations": STATIONS, "book_size": BOOK_SIZE,
                   "slot_clear_width_m": 0.165, "return_neighbors_y_m": [-0.15, 0.05], "shelf_neighbors": "fixed supporting books", "floor_z": -0.04}
