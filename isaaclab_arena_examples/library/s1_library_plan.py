# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Define library inventory and explicit contact-safe single and dual-book workflows."""

import copy


def catalog_for(batch_books=1):
    """Return independent book records for one or two returns and loans.

    Args:
        batch_books: Number of returned and borrowed books, each.

    Returns:
        Catalog with physical locations and per-book evaluation phases.
    """
    assert batch_books in (1, 2)
    catalog = {
        "return_book": dict(
            id="LIB-001",
            source=(0.85, -2.10, 0.97),
            target=(1.35, 0, 0.97),
            slot="A-01",
            side="left",
            withdraw_phase="return_withdraw",
            surface_z=0.86,
            lift_phase="lift_return",
            source_surface_z=0.86,
            min_lift_m=0.10,
            grasp_phase="grip_return",
        ),
        "blocker": dict(
            id="NBR-001",
            source=(1.35, 0, 0.97),
            target=(1.35, -0.15, 0.97),
            slot="A-02",
            side="right",
            withdraw_phase="blocker_withdraw",
            surface_z=0.86,
            lift_phase="blocker_raise",
            source_surface_z=0.86,
            min_lift_m=0.02,
            grasp_phase="grip_blocker",
        ),
        "loan_book": dict(
            id="LIB-003",
            source=(1.35, 1.59, 1.63),
            target=(0.85, -1.55, 0.97),
            slot="B-03",
            side="right",
            withdraw_phase="checkout_clear",
            surface_z=0.86,
            lift_phase="loan_raise",
            source_surface_z=1.52,
            min_lift_m=0.02,
            grasp_phase="grip_loan",
        ),
    }
    if batch_books == 2:
        catalog.update({
            "return_book2": dict(
                id="LIB-002",
                source=(0.85, -2.20, 0.97),
                target=(1.35, -0.30, 0.97),
                slot="A-04",
                side="right",
                withdraw_phase="return2_withdraw",
                surface_z=0.86,
                lift_phase="lift_return2",
                source_surface_z=0.86,
                min_lift_m=0.10,
                grasp_phase="grip_return2",
            ),
            "loan_book2": dict(
                id="LIB-004",
                source=(1.35, 2.02, 1.63),
                target=(0.85, -1.45, 0.97),
                slot="B-04",
                side="left",
                withdraw_phase="checkout2_clear",
                surface_z=0.86,
                lift_phase="loan2_raise",
                source_surface_z=1.52,
                min_lift_m=0.02,
                grasp_phase="grip_loan2",
            ),
        })
    return copy.deepcopy(catalog)


def legacy_plan():
    """Return the original box-return-shelf-checkout sequence with collision-clearing exits."""
    plan = []

    def move(name, duration=4, side=None, book=None, xyz=None, action=None, base=None, lift=None):
        plan.append(
            dict(
                name=name,
                duration=duration,
                side=side,
                book=book,
                xyz=xyz,
                action=action,
                base=base,
                lift=lift,
            )
        )

    move("settle", 3)
    move("return_reach", side="left", book="return_book", xyz=(0.85, -2.10, 0.97))
    move("grip_return", 2, "left", "return_book", action="close")
    move("lift_return", 3, "left", "return_book", (0.85, -2.10, 1.20))
    move("stow_return", 4, "left", "return_book", (0.68, -2.06, 1.25))
    move("drive_shelf_a_aisle", base=(-0.18, 0))
    move("drive_shelf_a", base=(0.30, 0))
    move("blocker_approach", side="right", book="blocker", xyz=(1.15, 0, 0.97), action="align")
    move("blocker_align", 2)
    move("blocker_reach", side="right", book="blocker", xyz=(1.35, 0, 0.97), action="align")
    move("grip_blocker", 2, "right", "blocker", action="close")
    move("blocker_raise", 3, "right", "blocker", (1.35, 0, 1.04))
    move("blocker_upright", 3, "right", "blocker", action="upright")
    move("blocker_extract", 3, "right", "blocker", (1.15, 0, 1.04))
    move("blocker_shift", 3, "right", "blocker", (1.15, -0.15, 1.04))
    move("blocker_park", 3, "right", "blocker", (1.35, -0.15, 1.04))
    move("blocker_seat", 3, "right", "blocker", (1.35, -0.15, 0.972))
    move("release_blocker", 2, "right", "blocker", action="open")
    move("blocker_withdraw", 3, "right", "blocker", (1.15, -0.15, 0.972))
    move("right_stow", 3, "right", xyz=(1.10, -0.25, 1.10))
    move("return_align", 4, "left", "return_book", (1.15, 0, 1.20))
    move("return_insert", 4, "left", "return_book", (1.35, 0, 1.10))
    move("return_seat", 3, "left", "return_book", (1.35, 0, 0.972))
    move("release_return", 2, "left", "return_book", action="open")
    move("return_withdraw", 3, "left", "return_book", (1.15, 0, 0.972))
    move("left_stow", 3, "left", xyz=(1.10, 0.25, 1.10))
    move("exit_shelf_a", base=(-0.18, 0))
    move("drive_shelf_b_aisle", base=(-0.18, 1.80))
    move("drive_shelf_b", base=(0.30, 1.80))
    move("raise_for_shelf_b", 5, lift=0.65)
    move("loan_approach", 4, "right", "loan_book", (1.15, 1.59, 1.63))
    move("loan_reach", 4, "right", "loan_book", (1.35, 1.59, 1.63))
    move("grip_loan", 2, "right", "loan_book", action="close")
    move("loan_raise", 3, "right", "loan_book", (1.35, 1.59, 1.70))
    move("loan_extract", 4, "right", "loan_book", (1.15, 1.59, 1.70))
    move("exit_shelf_b", base=(-0.18, 1.80))
    move("loan_stow", 4, "right", "loan_book", (0.68, 1.55, 1.50))
    move("lower_for_travel", 5, lift=0.25)
    move("drive_checkout", base=(-0.18, -1.345))
    move("checkout_approach", 4, "right", "loan_book", (0.85, -1.55, 1.15))
    move("checkout_seat", 4, "right", "loan_book", (0.85, -1.55, 0.972))
    move("release_loan", 2, "right", "loan_book", action="open")
    move("checkout_clear", 4, "right", "loan_book", (0.65, -1.55, 0.972))
    move("done", 3)
    return plan


def full_plan(batch_books=1, dense_shelves=False, motion_speed=1.0, packed_shelves=False):
    """Return one round trip, carrying one or two books in each direction.

    Args:
        batch_books: Number of returned and borrowed books, each.
        dense_shelves: Reserve the right hand for shorter shelf clearing.
        motion_speed: Arm trajectory speed multiplier; grasp and release waits stay unchanged.
        packed_shelves: Clear a uniformly stocked row by physical contact, without a parking slot.

    Returns:
        Named phases with motor trajectory, grasp and wheel-navigation goals.
    """
    assert batch_books in (1, 2)
    assert 1 <= motion_speed <= 2
    assert not dense_shelves or batch_books == 1, "Reserve the right hand for shelf clearing."
    plan = legacy_plan()
    for phase in plan:
        if phase["action"] not in ("close", "open") and phase["name"] not in ("settle", "done", "blocker_align"):
            phase["duration"] /= motion_speed
        if dense_shelves and phase["name"] in ("blocker_shift", "blocker_park", "blocker_seat", "blocker_withdraw"):
            phase["xyz"] = (phase["xyz"][0], -0.09, phase["xyz"][2])
    if packed_shelves:
        assert batch_books == 1 and dense_shelves
        # Slide the right-hand book against the neighboring row, then back off
        # before release. No empty parking slot is authored in the initial shelf.
        remove = {"blocker_upright", "blocker_extract"}
        plan = [p for p in plan if p["name"] not in remove]
        for p in plan:
            if p["name"] == "blocker_raise":
                p["xyz"] = (1.35, 0, 1.00)
            elif p["name"] == "blocker_shift":
                p["xyz"] = (1.35, -0.060, 0.972)
            elif p["name"] in ("blocker_park", "blocker_seat"):
                p["xyz"] = (1.35, -0.042, 0.972)
            elif p["name"] == "blocker_withdraw":
                p["xyz"] = (1.15, -0.042, 0.972)
            p["jaw_opening"] = 0.021
        index = next(i for i, p in enumerate(plan) if p["name"] == "blocker_shift")
        seat = dict(plan[index], name="blocker_slide_start", xyz=(1.35, 0, 0.972))
        plan.insert(index, seat)
    if batch_books == 1:
        return plan

    def phase(name, duration=4, side=None, book=None, xyz=None, action=None):
        if action not in ("close", "open"):
            duration /= motion_speed
        return dict(name=name, duration=duration, side=side, book=book, xyz=xyz, action=action, base=None, lift=None)

    def after(name, phases):
        index = next(i for i, p in enumerate(plan) if p["name"] == name) + 1
        plan[index:index] = phases

    after(
        "stow_return",
        [
            phase("return2_align_box", 3, "right", "return_book2", (0.65, -2.20, 1.05)),
            phase("return2_reach", 4, "right", "return_book2", (0.85, -2.20, 0.97)),
            phase("grip_return2", 2, "right", "return_book2", action="close"),
            phase("lift_return2", 3, "right", "return_book2", (0.85, -2.20, 1.20)),
            phase("stow_return2", 4, "right", "return_book2", (0.68, -2.53, 1.25)),
        ],
    )
    after(
        "drive_shelf_a",
        [
            phase("return2_align_shelf", 4, "right", "return_book2", (1.15, -0.30, 1.20)),
            phase("return2_insert", 4, "right", "return_book2", (1.35, -0.30, 1.10)),
            phase("return2_seat", 3, "right", "return_book2", (1.35, -0.30, 0.972)),
            phase("release_return2", 2, "right", "return_book2", action="open"),
            phase("return2_withdraw", 3, "right", "return_book2", (1.15, -0.30, 0.972)),
            phase("return2_clear", 3, "right", xyz=(1.10, -0.25, 1.10)),
        ],
    )
    after(
        "loan_extract",
        [
            phase("loan2_approach", 4, "left", "loan_book2", (1.15, 2.02, 1.63)),
            phase("loan2_reach", 4, "left", "loan_book2", (1.35, 2.02, 1.63)),
            phase("grip_loan2", 2, "left", "loan_book2", action="close"),
            phase("loan2_raise", 3, "left", "loan_book2", (1.35, 2.02, 1.70)),
            phase("loan2_extract", 4, "left", "loan_book2", (1.15, 2.02, 1.70)),
        ],
    )
    after("loan_stow", [phase("loan2_stow", 4, "left", "loan_book2", (0.68, 2.05, 1.50))])
    after(
        "checkout_clear",
        [
            phase("checkout_right_park", 3, "right", xyz=(0.65, -1.80, 1.20)),
            phase("checkout2_approach", 4, "left", "loan_book2", (0.85, -1.45, 1.15)),
            phase("checkout2_seat", 4, "left", "loan_book2", (0.85, -1.45, 0.972)),
            phase("release_loan2", 2, "left", "loan_book2", action="open"),
            phase("checkout2_clear", 4, "left", "loan_book2", (0.65, -1.45, 0.972)),
        ],
    )
    return plan
