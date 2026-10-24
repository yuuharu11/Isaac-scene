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
    # Quiet reading corner, windows and an aisle runner make room scale legible.
    box("AisleRunner", (1.55, 5.1, 0.002), (-0.95, -0.25, -0.038), (0.23, 0.35, 0.36), False)
    for side in (-1, 1):
        box(
            f"RunnerBorder{side+1}",
            (0.022, 5.1, 0.003),
            (-0.95 + side * 0.74, -0.25, -0.037),
            (0.66, 0.62, 0.47),
            False,
        )
    box("ReadingTable", (0.90, 0.55, 0.05), (-1.45, 2.65, 0.73), wood)
    for ix, x in enumerate((-1.80, -1.10)):
        for iy, y in enumerate((2.45, 2.85)):
            box(f"ReadingLeg{ix}{iy}", (0.045, 0.045, 0.72), (x, y, 0.32), (0.26, 0.27, 0.25))
    for i, x in enumerate((-1.4, -0.15, 1.1)):
        box(f"WindowFrame{i}", (0.97, 0.025, 0.98), (x, 3.195, 1.66), wood, False)
        box(f"WindowGlass{i}", (0.89, 0.012, 0.90), (x, 3.176, 1.66), (0.65, 0.79, 0.82), False)
        box(f"WindowMullion{i}", (0.025, 0.015, 0.90), (x, 3.166, 1.66), (0.85, 0.84, 0.77), False)
    return dict(
        reading_corner=True,
        aisle_runner=True,
        shelves=7,
        added_shelves=len(cases),
        added_static_books=count,
        background_books_static=True,
        working_neighbors_dynamic=True,
        left_furniture_max_x_m=-2.34,
        working_base_min_x_m=-0.18,
        working_base_max_x_m=0.30,
    )
