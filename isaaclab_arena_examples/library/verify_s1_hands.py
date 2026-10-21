# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Verify thumbs-up anatomy and inward finger closure from measured S1 captures."""

import argparse
import json
from pathlib import Path


def verify(directory):
    """Return measured orientation checks from a completed seven-phase capture.

    Args:
        directory: Directory containing scene.npz.

    Returns:
        Per-side anatomical measurements and their combined pass flag.
    """
    import numpy as np

    measurements = {}
    with np.load(Path(directory) / "scene.npz", allow_pickle=False) as scene:
        frames = {phase: int(np.flatnonzero(scene["phases"] == phase)[-1])
                  for phase in ("home", "close_hands", "open_hands")}
        for side, inward_sign in (("left", -1), ("right", 1)):
            def body(phase, name):
                index = next(i for i, path in enumerate(scene["body_paths"]) if path.endswith("/" + side + name))
                return scene["poses"][frames[phase], index, :3]

            thumb_height = float(body("home", "_hand_virtual_a")[2] - body("home", "_hand_base_link")[2])
            inward_closure = float(inward_sign * (body("close_hands", "_hand_r")[1]
                                                 - body("open_hands", "_hand_r")[1]))
            measurements[side] = {"thumb_side_above_wrist_m": thumb_height,
                                  "inward_finger_closure_m": inward_closure,
                                  "success": thumb_height > 0.01 and inward_closure > 0.02}
    return {"success": all(value["success"] for value in measurements.values()), "hands": measurements}


def main():
    """Write an anatomical verification report and fail for reversed hand mounts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    args = parser.parse_args()
    result = verify(args.capture_dir)
    (args.capture_dir / "hand-orientation.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    assert result["success"], "Thumbs must point up and both hands must close toward the body center."


if __name__ == "__main__":
    main()
