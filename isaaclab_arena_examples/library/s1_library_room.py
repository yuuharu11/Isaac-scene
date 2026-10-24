# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Add densely stocked library furniture outside the robot's working aisle."""


def add_library_atmosphere(box):
    """Build five additional static bookcases and stock unused original shelf levels.

    Args:
        box: Scene cuboid factory accepting name, size, xyz, color and collision.

    Returns:
        Furniture inventory and aisle-clearance design bounds.
    """
    wood = (0.38, 0.24, 0.13)
    palette = [
        (0.12, 0.25, 0.37),
        (0.35, 0.16, 0.13),
        (0.26, 0.36, 0.24),
        (0.44, 0.31, 0.17),
        (0.28, 0.25, 0.36),
        (0.55, 0.50, 0.37),
    ]
    count = 0

    def row(prefix, x, cy, surface, count_books, facing=-1):
        nonlocal count
        pitch = 0.042
        for j in range(count_books):
            width = (0.030, 0.033, 0.036, 0.032)[j % 4]
            height = (0.23, 0.24, 0.21, 0.25, 0.22)[j % 5]
            y = cy + (j - (count_books - 1) / 2) * pitch
            z = surface + height / 2
            name = f"{prefix}Book{j}"
            color = palette[(j + len(prefix)) % len(palette)]
            box(name, (0.17, width, height), (x, y, z), color)
            if j % 3 == 0:
                box(
                    name + "Label",
                    (0.001, width * 0.7, 0.025),
                    (x + facing * 0.0857, y, z - height * 0.24),
                    (0.82, 0.80, 0.70),
                    False,
                )
            count += 1

    cases = [(-2.55, y, 1) for y in (-2.65, -1.20, 0.25, 1.70)] + [(1.36, -2.85, -1)]
    for index, (x, y, facing) in enumerate(cases):
        prefix = f"AmbientShelf{index}"
        box(prefix + "Back", (0.025, 1.08, 2.14), (x - facing * 0.205, y, 1.03), wood)
        for side in (-1, 1):
            box(prefix + f"Side{side+1}", (0.42, 0.025, 2.14), (x, y + side * 0.54, 1.03), wood)
        for level, surface in enumerate((0.24, 0.63, 1.02, 1.41, 1.80)):
            box(prefix + f"Board{level}", (0.42, 1.08, 0.03), (x, y, surface - 0.015), wood)
            row(prefix + f"Row{level}", x, y, surface, 22, facing)
        box(prefix + "Top", (0.42, 1.08, 0.03), (x, y, 2.10), wood)
    for letter, cy, levels in (("A", 0, (0.20, 0.53)), ("B", 1.8, (0.20, 1.11))):
        for index, surface in enumerate(levels):
            prefix = f"Shelf{letter}ExtraRow{index}"
            box(prefix + "Board", (0.40, 1.0, 0.025), (1.36, cy, surface - 0.0125), wood)
            row(prefix, 1.35, cy, surface, 18)
    return dict(
        shelves=7,
        added_shelves=len(cases),
        added_static_books=count,
        background_books_static=True,
        working_neighbors_dynamic=True,
        left_furniture_max_x_m=-2.34,
        working_base_min_x_m=-0.18,
        working_base_max_x_m=0.30,
    )
