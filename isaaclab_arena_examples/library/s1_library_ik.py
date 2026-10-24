# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Solve S1 wrist poses from its official URDF within authored joint limits."""

import numpy as np
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class S1Kinematics:
    """Provide bounded pose IK for the two seven-axis S1 arms."""

    def __init__(self):
        asset = Path(os.environ.get("GALBOT_S1_USD", "/work/data/galbot_s1_description/usd/galbot_s1.usda"))
        tree = ET.parse(asset.parent.parent / "urdf/galbot_s1.urdf").getroot()
        self.joints = {j.find("child").get("link"): j for j in tree.findall("joint")}
        self.by_name = {j.get("name"): j for j in tree.findall("joint")}
        self.chains = {}
        for side in ("left", "right"):
            chain, link = [], side + "_arm_link7"
            while link in self.joints:
                j = self.joints[link]
                origin = j.find("origin")
                transform = np.eye(4)
                if origin is not None:
                    transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                    transform[:3, :3] = Rotation.from_euler(
                        "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                    ).as_matrix()
                axis = j.find("axis")
                chain.append((
                    j.get("name"),
                    j.get("type"),
                    transform,
                    np.fromstring(axis.get("xyz", "1 0 0"), sep=" ") if axis is not None else np.array([1.0, 0.0, 0.0]),
                ))
                link = j.find("parent").get("link")
            self.chains[side] = chain[::-1]
        self.goal_rotation = Rotation.from_euler("z", np.pi).as_matrix()

    def forward(self, side, values, lift):
        """Return the wrist transform for seven joint values and a lift height.

        Args:
            side: Left or right arm.
            values: Seven arm joint angles in radians.
            lift: Torso lift in metres.

        Returns:
            Wrist transform in the chassis frame.
        """
        q = dict(zip([f"{side}_arm_joint{i}" for i in range(1, 8)], values)) | {"torso_lift_joint1": lift}
        world = np.eye(4)
        for name, kind, origin, axis in self.chains[side]:
            motion = np.eye(4)
            if kind == "prismatic":
                motion[:3, 3] = axis * q.get(name, 0.0)
            elif kind in ("continuous", "revolute"):
                motion[:3, :3] = Rotation.from_rotvec(axis * q.get(name, 0.0)).as_matrix()
            world = world @ origin @ motion
        return world

    def solve(self, side, position, lift, seed, rotation=None):
        """Return bounded joint targets for a forward-facing wrist pose.

        Args:
            side: Left or right arm.
            position: Wrist XYZ in the chassis frame, in metres.
            lift: Torso lift in metres.
            seed: Previous seven arm joint targets.
            rotation: Optional wrist rotation matrix in the chassis frame.

        Returns:
            Seven joint targets and their position error in metres.
        """
        names = [f"{side}_arm_joint{i}" for i in range(1, 8)]
        bounds = np.array([
            (
                float(self.by_name[n].find("limit").get("lower")) + 0.01,
                float(self.by_name[n].find("limit").get("upper")) - 0.01,
            )
            for n in names
        ]).T
        seed = np.asarray(seed)
        goal_rotation = self.goal_rotation if rotation is None else np.asarray(rotation)

        def residual(q):
            pose = self.forward(side, q, lift)
            return np.r_[
                10 * (pose[:3, 3] - position),
                Rotation.from_matrix(goal_rotation.T @ pose[:3, :3]).as_rotvec(),
                0.002 * (q - seed),
            ]

        result = least_squares(residual, np.clip(seed, *bounds), bounds=tuple(bounds), max_nfev=150)
        pose = self.forward(side, result.x, lift)
        error = float(np.linalg.norm(pose[:3, 3] - position))
        angle = float(Rotation.from_matrix(goal_rotation.T @ pose[:3, :3]).magnitude())
        assert (
            error < 0.006 and angle < 0.04
        ), f"Unreachable {side} wrist: {position}, errors={error:.4f}m/{angle:.3f}rad"
        return result.x, error
