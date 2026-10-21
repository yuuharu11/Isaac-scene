# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Render measured Galbot bring-up at the capture's real-time sampling rate."""

import argparse
import json
from pathlib import Path

from isaaclab_arena_examples.library.galbot_robot import TORSO_BODY
from isaaclab_arena_examples.library.render_video import LibraryRenderer


def main():
    """Create an MP4 and phase previews from the unchanged measured pose archive."""
    import imageio_ffmpeg
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--preview-only", action="store_true")
    args = parser.parse_args()
    directory = args.capture_dir
    report = json.loads((directory / "result.json").read_text())
    capture = json.loads((directory / "capture.json").read_text())
    fps = capture["sampling_fps"]
    width, height = 960, 720
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 21)
    labels = {
        "home": "Official Galbot | neutral posture",
        "lower_torso": "Lower the torso through the official leg joints",
        "move_left_arm": "Move the left arm while holding torso height",
        "move_both_arms": "Move the right arm while holding the left arm",
        "raise_torso_and_return": "Raise the torso and return both arms smoothly",
    }
    with np.load(directory / "scene.npz", allow_pickle=False) as scene:
        renderer = LibraryRenderer(scene, width, height)
        renderer.camera_eye = [2.3, -2.1, 1.55]
        renderer.camera_target = [0.1, 0.0, 0.82]
        renderer.fov = 45.0
        renderer.set_camera()
        torso = next(i for i, path in enumerate(scene["body_paths"]) if path.endswith("/" + TORSO_BODY))

        def frame(index):
            phase = str(scene["phases"][index])
            image = Image.fromarray(renderer.render(scene["poses"][index]))
            draw = ImageDraw.Draw(image)
            title = labels[phase]
            if not report["success"]:
                title = "UNVERIFIED TRIAL | " + title
            z = scene["poses"][index, torso, 2]
            text = (
                f"{title}\nTorso Z: {z:.3f} m | t={scene['times'][index]:.1f}s | real time\n"
                "Control bench: fixed chassis, no gravity or self collisions"
            )
            draw.rectangle((0, height - 96, width, height), fill=(22, 29, 34))
            draw.multiline_text((12, height - 90), text, font=font, fill=(240, 244, 247), spacing=5)
            return np.asarray(image)

        for phase in labels:
            indices = np.flatnonzero(scene["phases"] == phase)
            if len(indices):
                Image.fromarray(frame(int(indices[-1]))).save(directory / (phase + ".png"))
        if args.preview_only:
            return
        output = directory / "galbot-bringup.mp4"
        writer = imageio_ffmpeg.write_frames(
            str(output), (width, height), fps=fps, codec="libx264", quality=8,
            pix_fmt_in="rgb24", pix_fmt_out="yuv420p", macro_block_size=1,
            ffmpeg_log_level="error", output_params=["-movflags", "+faststart"],
        )
        writer.send(None)
        try:
            for index in range(len(scene["poses"])):
                writer.send(frame(index))
                if index % 200 == 0:
                    print(f"Rendered {index + 1}/{len(scene['poses'])}", flush=True)
        finally:
            writer.close()
        metadata = {
            "frames": len(scene["poses"]), "fps": fps, "width": width, "height": height,
            "duration_seconds": len(scene["poses"]) / fps, "simulation_success": report["success"],
            "renderer": "USD geometry and measured PhysX poses; CUDA rasterization, not RTX footage",
            "speedup": 1.0,
        }
        output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(json.dumps(metadata), flush=True)


if __name__ == "__main__":
    main()
