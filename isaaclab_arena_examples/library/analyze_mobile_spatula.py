# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Verify the complete measured wheel-driven, contact-grasp library workflow."""

import argparse
import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation


def bottom(pose):
    """Return the lowest point of a measured book cuboid."""
    return float(pose[2] - np.abs(Rotation.from_quat(pose[3:]).as_matrix()[2]) @ [0.08, 0.0175, 0.11])


def unexpected_tool_contacts(contacts):
    """Return tool contacts with the robot or the opposing tool articulation.

    Args:
        contacts: Collider-pair summaries from the physical contact subscription.

    Returns:
        Unintended tool/arm and left/right tool contact summaries.
    """
    return [
        pair
        for pair in contacts.values()
        if any("/Tool" in actor for actor in pair["actors"])
        and (
            any("/Galbot/" in actor for actor in pair["actors"])
            or (
                any(actor.startswith("/World/ToolLeft") for actor in pair["actors"])
                and any(actor.startswith("/World/ToolRight") for actor in pair["actors"])
            )
        )
    ]


def analyze(directory, finalize=False):
    """Score recorded contacts and motion before optionally promoting the result metadata.

    Args:
        directory: Completed or failed full-scene capture.
        finalize: Preserve the runtime report and write the verified success into result/capture metadata.

    Returns:
        Verification report; measured poses are never modified.
    """
    from isaaclab_arena_examples.library.s1_library_plan import full_plan

    report = json.loads((directory / "result.json").read_text())
    trace = report["trace"]
    phases = list(dict.fromkeys(r["phase"] for r in trace))
    expected = report.get("expected_plan") or full_plan(1)
    catalog = report["scene"]["catalog"]
    audit = report.get("motion_audit")
    audit_path = directory / "motion_audit.json"
    if audit is None and audit_path.exists():
        audit = json.loads(audit_path.read_text())
    legacy = {
        "return_book": dict(
            side="left",
            withdraw_phase="return_withdraw",
            surface_z=0.86,
            lift_phase="lift_return",
            source_surface_z=0.86,
            min_lift_m=0.10,
        ),
        "blocker": dict(
            side="right",
            withdraw_phase="blocker_withdraw",
            surface_z=0.86,
            lift_phase="blocker_raise",
            source_surface_z=0.86,
            min_lift_m=0.02,
        ),
        "loan_book": dict(
            side="right",
            withdraw_phase="checkout_clear",
            surface_z=0.86,
            lift_phase="loan_raise",
            source_surface_z=1.52,
            min_lift_m=0.02,
            grasp_phase="grip_loan",
        ),
    }
    metadata = {name: {**legacy.get(name, {}), **entry} for name, entry in catalog.items()}
    complete = phases == [p["name"] for p in expected] and report["error"] is None and report["stopped_after"] is None
    rows = {name: [r for r in trace if r["phase"] == name] for name in phases}
    final = trace[-1]
    books = {}
    for name, entry in metadata.items():
        side, withdraw, surface = entry["side"], entry["withdraw_phase"], entry["surface_z"]
        target = report["scene"]["catalog"][name]["target"]
        p = np.asarray(final["books"][name])
        position_error = float(np.linalg.norm(p[:3] - target))
        seat_error = abs(bottom(p) - surface)
        tilt = float(np.degrees(np.arccos(np.clip(Rotation.from_quat(p[3:]).as_matrix()[2, 2], -1, 1))))
        velocity = np.asarray(final.get("book_velocities", {}).get(name, [float("inf")] * 6))
        stable = bool(np.linalg.norm(velocity[:3]) < 0.03 and np.linalg.norm(velocity[3:]) < 0.1 and tilt < 15)
        held = [r for r in trace if r["tools"][side]["held"] == name and not r["phase"].startswith("release_")]
        contact = 0.0
        slip = float("inf")
        if held:
            contact = float(np.mean([min(r["tools"][side]["forces"][name]) > 0.05 for r in held]))
            relative = [
                Rotation.from_quat(r["tools"][side]["pose"][3:])
                .inv()
                .apply(np.asarray(r["books"][name][:3]) - r["tools"][side]["pose"][:3])
                for r in held
            ]
            slip = float(np.max(np.linalg.norm(np.asarray(relative) - relative[0], axis=1)))
        continuous = audit.get("books", {}).get(name) if audit else None
        held_bottom = min((bottom(r["books"][name]) for r in held), default=None)
        if audit and (not continuous or not continuous.get("held_samples")):
            contact, slip = 0.0, float("inf")
        if continuous:
            contact = float(continuous["both_jaws_contact_fraction"])
            slip = float(continuous.get("max_relative_slip_m", float("inf")))
            held_bottom = continuous.get("min_held_bottom_m")
        low_held = bool(
            held_bottom is not None and held_bottom < min(surface, entry.get("source_surface_z", surface)) - 0.005
        )
        released = all(max(t["forces"][name]) < 0.05 and t["held"] != name for t in final["tools"].values())
        withdrawn = False
        clearance = None
        if withdraw in rows:
            r = rows[withdraw][-1]
            t = r["tools"][side]["pose"]
            b = r["books"][name]
            rotation = Rotation.from_quat(t[3:]).as_matrix()
            tip = t[0] + rotation[0, 0] * 0.20 + np.abs(rotation[0]) @ [0.06, 0.0615, 0.04]
            front = b[0] - np.abs(Rotation.from_quat(b[3:]).as_matrix()[0]) @ [0.08, 0.0175, 0.11]
            clearance = float(front - tip)
            withdrawn = clearance > 0.02
        books[name] = dict(
            position_error_m=position_error,
            shelf_or_box_bottom_error_m=seat_error,
            tilt_deg=tilt,
            stable=stable,
            both_jaws_contact_fraction=contact,
            max_relative_slip_m=slip,
            released=bool(released),
            blades_withdrawn=bool(withdrawn),
            blade_front_clearance_m=clearance,
            success=bool(
                position_error < 0.025
                and seat_error < 0.005
                and stable
                and contact > 0.8
                and slip < 0.01
                and released
                and withdrawn
                and (not audit or not low_held)
            ),
        )
        if audit:
            books[name].update(
                min_held_bottom_m=held_bottom,
                held_below_support=low_held,
                post_withdraw_contact_samples=(
                    continuous.get("post_withdraw_contact_samples", 0) if continuous else None
                ),
                continuous_held_samples=continuous.get("held_samples", 0) if continuous else 0,
            )
    lift_requirements = {
        entry["lift_phase"]: (name, entry["source_surface_z"], entry["min_lift_m"]) for name, entry in metadata.items()
    }
    lifts = {
        name: float(bottom(rows[name][-1]["books"][book]) - surface) if name in rows else None
        for name, (book, surface, minimum) in lift_requirements.items()
    }
    if audit:
        for phase, (name, surface, _) in lift_requirements.items():
            measured = audit.get("phase_measurements", {}).get(phase, {}).get(name)
            if measured:
                lifts[phase] = float(measured["final_bottom_m"] - surface)
    lifted = all(
        lifts[name] is not None and lifts[name] > minimum for name, (_, _, minimum) in lift_requirements.items()
    )
    navigation = report["navigation"]
    navigation_ok = [p["name"] for p in expected if p["base"] is not None] == [e["phase"] for e in navigation] and all(
        e["error_m"] < 0.006 and e["quiet_s"] > 0.7 for e in navigation
    )
    path = np.asarray([r["base_pose"][:2] for r in trace])
    distance = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    contacts = json.loads((directory / "contact_pairs.json").read_text())
    unintended = unexpected_tool_contacts(contacts)
    tool_contacts = [c for c in contacts.values() if any("/Tool" in actor for actor in c["actors"])]
    penetration = float(max((c["max_penetration_m"] for c in tool_contacts), default=float("inf")))
    neighbors = float(audit["max_neighbor_displacement_m"] if audit else report["max_neighbor_displacement_m"])
    clearance_checks = {}
    all_neighbors_displacement = neighbors
    planned = set(report["scene"].get("clearance_neighbors", []))
    if planned:
        assert audit, "Packed row clearing requires the continuous neighbor audit."
        neighbors = max(
            (entry["max_displacement_m"] for name, entry in audit["neighbors"].items() if name not in planned),
            default=0.0,
        )
        for name in sorted(planned):
            entry = audit["neighbors"][name]
            initial, final_pose = np.asarray(entry["initial_pose"]), np.asarray(entry["final_pose"])
            velocity = np.asarray(entry["final_velocity"])
            delta = final_pose[:3] - initial[:3]
            support_error = abs(bottom(final_pose) - 0.86)
            success = bool(
                entry["max_displacement_m"] < report["scene"]["clearance_neighbor_motion_limit_m"]
                and entry["max_rotation_deg"] < report["scene"]["clearance_neighbor_rotation_limit_deg"]
                and abs(delta[0]) < 0.005
                and -0.070 < delta[1] < 0.005
                and support_error < 0.005
                and np.linalg.norm(velocity[:3]) < 0.03
                and np.linalg.norm(velocity[3:]) < 0.1
            )
            clearance_checks[name] = dict(
                success=success,
                displacement_m=delta.tolist(),
                support_error_m=support_error,
                max_displacement_m=entry["max_displacement_m"],
                max_rotation_deg=entry["max_rotation_deg"],
            )
    high_grasp_by_book = {
        name: bool(
            entry.get("grasp_phase") in rows
            and rows[entry["grasp_phase"]][-1]["books"][name][2]
            > (1.60 if entry["source_surface_z"] == 1.52 else entry["source_surface_z"] + 0.08)
        )
        for name, entry in metadata.items()
        if entry.get("source_surface_z", 0) > 1.0
    }
    if audit:
        for name in high_grasp_by_book:
            entry = metadata[name]
            measured = audit.get("phase_measurements", {}).get(entry["grasp_phase"], {}).get(name)
            if measured:
                high_grasp_by_book[name] = measured["final_center_z_m"] > (
                    1.60 if entry["source_surface_z"] == 1.52 else entry["source_surface_z"] + 0.08
                )
    high_grasp = bool(high_grasp_by_book) and all(high_grasp_by_book.values())
    success = bool(
        complete
        and all(b["success"] for b in books.values())
        and lifted
        and navigation_ok
        and distance > 8
        and penetration < 0.0005
        and neighbors < 0.005
        and all(entry["success"] for entry in clearance_checks.values())
        and high_grasp
        and not unintended
    )
    result = dict(
        success=success,
        complete_original_workflow=complete,
        books=books,
        lift_clearance_m=lifts,
        navigation_settled=navigation_ok,
        measured_base_path_m=distance,
        max_working_neighbor_displacement_m=neighbors,
        max_all_neighbor_displacement_m=all_neighbors_displacement,
        planned_clearance_neighbors=clearance_checks,
        unexpected_tool_contacts=unintended,
        max_reported_tool_penetration_m=penetration,
        high_shelf_grasp=high_grasp,
        physics="PhysX",
        base_drive=report["base_drive"],
        book_grasp=report["book_grasp"],
        self_collisions=report["self_collisions"],
        criteria=dict(
            max_final_position_error_m=0.025,
            max_seat_error_m=0.005,
            max_book_slip_m=0.01,
            min_bilateral_contact_fraction=0.8,
            max_neighbor_displacement_m=0.005,
            max_unplanned_neighbor_displacement_m=0.005,
            planned_clearance_neighbor_motion_limit_m=report["scene"].get("clearance_neighbor_motion_limit_m"),
            planned_clearance_neighbor_rotation_limit_deg=report["scene"].get("clearance_neighbor_rotation_limit_deg"),
            max_reported_tool_penetration_m=0.0005,
            max_station_error_m=0.006,
            min_station_quiet_s=0.7,
        ),
        limitations=report["limitations"],
    )
    if report.get("batch_books", 1) == 2:
        result["unexpected_tool_contacts"] = unintended
        result["tool_furniture_contacts"] = [
            pair
            for pair in contacts.values()
            if any("/Tool" in actor for actor in pair["actors"])
            and any("/Library/" in actor for actor in pair["actors"])
        ]
    if audit:
        result.update(
            motion_audit_sampling_dt_s=audit.get("sampling_dt_s"),
            high_shelf_grasp_by_book=high_grasp_by_book,
            neighbor_motion=audit.get("neighbors", {}),
            jaw_force_measurements=audit.get("forces", {}),
            body_furniture_contacts=[
                pair
                for pair in contacts.values()
                if any("/Library/" in actor for actor in pair["actors"])
                and any("/Galbot/" in actor or actor.startswith("/World/Book_") for actor in pair["actors"])
            ],
            target_floor_contacts=[
                pair
                for pair in contacts.values()
                if any(actor.startswith("/World/Book_") for actor in pair["actors"])
                and any("Floor" in collider or "Ground" in collider for collider in pair.get("colliders", []))
            ],
        )
    (directory / "verification.json").write_text(json.dumps(result, indent=2) + "\n")
    if finalize:
        capture_path = directory / "capture.json"
        capture = json.loads(capture_path.read_text())
        raw = directory / "runtime_result.json"
        if not raw.exists() and not capture.get("full_metadata_archive"):
            raw.write_text(json.dumps(report, indent=2) + "\n")
        report.update(success=success, verification="verification.json", verification_status="complete")
        (directory / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        capture["simulation_result"] = (
            {key: value for key, value in report.items() if key != "trace"} if capture.get("trace_ref") else report
        )
        capture_path.write_text(json.dumps(capture, indent=2) + "\n")
    return result


def main():
    """Verify a completed full-scene capture without re-running physics."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--finalize", action="store_true")
    args = parser.parse_args()
    print(json.dumps(analyze(args.directory, args.finalize), indent=2))


if __name__ == "__main__":
    main()
