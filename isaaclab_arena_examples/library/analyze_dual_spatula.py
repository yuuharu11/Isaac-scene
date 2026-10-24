# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Verify actual lift clearance, shelf seating and per-step contact penetration."""

import argparse
import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation


def bottom(pose):
    """Return the lowest point of a 160 by 35 by 220 mm measured book."""
    pose = np.asarray(pose)
    return float(pose[2] - np.abs(Rotation.from_quat(pose[3:]).as_matrix()[2]) @ [0.08, 0.0175, 0.11])


def analyze(directory):
    """Write lift, release, neighbor and contact checks for a completed two-jaw run.

    Args:
        directory: Completed experiment directory.

    Returns:
        Per-case verification of the recorded physical experiment.
    """
    report = json.loads((directory / "result.json").read_text())
    contacts = json.loads((directory / "contact_pairs.json").read_text())
    cases = []
    for i, case in enumerate(report["cases"]):
        prefix = f"/World/Case{i}/"
        pairs = [p for p in contacts.values() if any(prefix in a for a in p["actors"])]
        overlap = max((p["max_penetration_m"] for p in pairs), default=0)
        lift = [r for r in case["trace"] if r["phase"] == "lift"][-1]
        clearance = bottom(lift["book_poses_xyzw"][1]) - 0.86
        seat_error = abs(bottom(case["final_book_poses_xyzw"][1]) - 0.86)
        baseline, final = np.array(case["baseline_book_poses_xyzw"]), np.array(case["final_book_poses_xyzw"])
        disturbance = float(max(np.linalg.norm(final[j, :3] - baseline[j, :3]) for j in (0, 2)))
        passed = bool(case["success"] and clearance >= 0.02 and seat_error <= 0.005 and overlap <= 0.0005)
        cases.append(
            dict(
                initial_angle_deg=case["initial_angle_deg"],
                settled_tilt_deg=case["settled_tilt_deg"],
                final_tilt_deg=case["final_tilt_deg"],
                shift_m=case["lateral_shift_m"],
                lift_clearance_m=clearance,
                final_seat_error_m=seat_error,
                max_neighbor_displacement_m=disturbance,
                max_reported_tool_penetration_m=overlap,
                both_jaws_contact_fraction=case["both_jaws_contact_fraction"],
                max_slip_m=case["max_book_slip_relative_to_carriage_m"],
                all_books_stable=case["all_books_stable"],
                blades_withdrawn=case["blades_withdrawn"],
                final_jaw_contact_N=case["final_jaw_contact_N"],
                success=passed,
            )
        )
        if report.get("return_book"):
            returned = dict(case["return"])
            before = np.asarray([r for r in case["trace"] if r["phase"] == "rest"][-1]["book_poses_xyzw"])
            returning = [r for r in case["trace"] if r["phase"].startswith("return_")]
            disturbance = float(
                max(
                    np.linalg.norm(np.asarray(r["book_poses_xyzw"])[:3, :3] - before[:3, :3], axis=1).max()
                    for r in returning
                )
            )
            depth_error = abs(returned["final_pose_xyzw"][0] - case["baseline_book_poses_xyzw"][1][0])
            returned.update(max_other_book_displacement_during_return_m=disturbance, shelf_depth_error_m=depth_error)
            returned["success"] = bool(returned["success"] and disturbance < 0.005 and depth_error < 0.01)
            cases[-1]["return"] = returned
            cases[-1]["success"] = bool(cases[-1]["success"] and returned["success"])
    result = dict(
        success=all(case["success"] for case in cases),
        cases=cases,
        contact_check="PhysX contact events at every integration step",
        scope=report["scope"],
        criteria=dict(
            min_lift_clearance_m=0.02,
            max_seat_error_m=0.005,
            max_reported_tool_penetration_m=0.0005,
            max_slip_m=0.01,
            min_both_jaws_contact_fraction=0.8,
        ),
        limitations=[
            "Contact events cover tool contacts; unreported robot self contacts are not included.",
            "No damage or deformation model is used.",
        ],
    )
    if report.get("return_book"):
        result["criteria"]["return"] = dict(
            min_lift_m=0.04,
            max_slip_m=0.01,
            min_both_jaws_contact_fraction=0.8,
            max_shelf_bottom_error_m=0.005,
            max_shelf_depth_error_m=0.01,
            max_slot_center_error_m=0.01,
            max_other_book_displacement_during_return_m=0.005,
            stable_released_and_blades_withdrawn=True,
        )
    (directory / "verification.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main():
    """Check one or more measured gripper runs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directories", nargs="+", type=Path)
    args = parser.parse_args()
    for directory in args.directories:
        print(directory)
        print(json.dumps(analyze(directory), indent=2))


if __name__ == "__main__":
    main()
