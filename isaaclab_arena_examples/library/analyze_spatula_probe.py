# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Check measured book clearances and sampled collision penetration for contact probes."""

import argparse
import itertools
import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation


def overlap_depth(a, half_a, b, half_b):
    """Return the separating-axis overlap of two oriented boxes in metres.

    Args:
        a: First XYZ/XYZW pose.
        half_a: First box half sizes.
        b: Second XYZ/XYZW pose.
        half_b: Second box half sizes.

    Returns:
        Positive minimum overlap when intersecting, or a negative separating margin.
    """
    ra, rb = (
        Rotation.from_quat(a[3:]).as_matrix(),
        Rotation.from_quat(b[3:]).as_matrix(),
    )
    axes = [*ra.T, *rb.T, *(np.cross(x, y) for x in ra.T for y in rb.T)]
    result = float("inf")
    for axis in axes:
        norm = np.linalg.norm(axis)
        if norm < 1e-9:
            continue
        axis = axis / norm
        extent = np.abs(axis @ ra) @ half_a + np.abs(axis @ rb) @ half_b
        result = min(result, float(extent - abs(axis @ (a[:3] - b[:3]))))
    return result


def analyze(directory):
    """Write geometric checks and useful-gap criteria for a measured probe archive.

    Args:
        directory: Directory containing result.json and scene.npz.

    Returns:
        Geometry checks and per-case task scores.
    """
    report = json.loads((directory / "result.json").read_text())
    scene = np.load(directory / "scene.npz", allow_pickle=False)
    paths = scene["body_paths"].tolist()
    book_half = np.array([0.08, 0.0175, 0.11])
    tool_half = np.array([0.06, report["thickness_m"] / 2, 0.02])
    results = []
    for index, case in enumerate(report["cases"]):
        ids = [paths.index(f"/World/Case{index}/Book{i}") for i in range(3)]
        ids.append(paths.index(f"/World/Case{index}/Tool"))
        halves = [book_half, book_half, book_half, tool_half]
        penetration, worst = 0.0, None
        for frame, time in enumerate(scene["times"]):
            poses = scene["poses"][frame, ids]
            for i, j in itertools.combinations(range(4), 2):
                depth = overlap_depth(poses[i], halves[i], poses[j], halves[j])
                if depth > penetration:
                    penetration = depth
                    worst = dict(time_s=float(time), bodies=[paths[ids[i]], paths[ids[j]]])
        final = np.array(case["final_book_poses_xyzw"])
        widths = np.array([np.abs(Rotation.from_quat(p[3:]).as_matrix()[1]) @ book_half for p in final])
        # Conservative width of a full-height vertical corridor between target and right neighbor.
        gap = float((final[2, 1] - widths[2]) - (final[1, 1] + widths[1]))
        useful = gap >= 0.041 and case["all_books_stable"]
        result = dict(
            initial_angle_deg=case["initial_angle_deg"],
            settled_tilt_deg=case["settled_tilt_deg"],
            final_tilt_deg=case["final_tilt_deg"],
            insertion_depth_m=case["insertion_depth_m"],
            displacement_m=case["target_displacement_along_push_m"],
            final_vertical_corridor_width_m=gap,
            required_corridor_width_m=0.041,
            useful_gap_opened=bool(useful),
            tidy_gap_opened=bool(useful and case["final_tilt_deg"] <= 15),
            sampled_max_book_blade_overlap_m=penetration,
            worst_overlap=worst,
            sampled_overlap_below_0_5mm=bool(penetration <= 0.0005),  # noqa: SIM901 - JSON needs a native bool.
        )
        results.append(result)
    verification = dict(
        cases=results,
        sampling_fps=20,
        limitations=[
            "Overlap check covers book/book and blade/book boxes at captured times only.",
            "It does not bound between-frame penetration, robot collisions, or the S1 shank.",
            "Vertical corridor is a conservative Y-envelope test; it is not a demonstrated insertion.",
        ],
    )
    (directory / "verification.json").write_text(json.dumps(verification, indent=2) + "\n", encoding="utf-8")
    return verification


def main():
    """Analyze one or more measured contact experiments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directories", type=Path, nargs="+")
    args = parser.parse_args()
    for directory in args.directories:
        print(str(directory))
        print(json.dumps(analyze(directory), indent=2))


if __name__ == "__main__":
    main()
