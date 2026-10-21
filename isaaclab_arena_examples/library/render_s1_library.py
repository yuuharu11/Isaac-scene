# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Render measured S1 library task poses with aisle-wide navigation views."""

import argparse
import json
from pathlib import Path


def main():
    """Render representative stills or encode the completed physical pose archive."""
    import imageio_ffmpeg
    import numpy as np
    from PIL import Image
    from isaaclab_arena_examples.library.render_video import LibraryRenderer

    class StableLibraryRenderer(LibraryRenderer):
        """Use matte shading without moving shadow-map edges in this presentation."""

        specular_strength = 0.0

        def visibility(self, world, position, normal):
            """Return uniform visibility for the shadow-free presentation lighting."""
            return self.torch.ones_like(position[..., :1])

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--preview-only", action="store_true")
    parser.add_argument("--speedup", type=float, default=1.5, help="Playback speed relative to recorded physics.")
    args = parser.parse_args()
    assert args.speedup > 0, "Playback speed must be positive."
    directory = args.capture_dir
    report = json.loads((directory / "result.json").read_text())
    fps = json.loads((directory / "capture.json").read_text())["sampling_fps"] * args.speedup
    width, height = 1920, 720
    with np.load(directory / "scene.npz", allow_pickle=False) as scene:
        renderer = StableLibraryRenderer(scene, width // 2, height)
        base = next(i for i, p in enumerate(scene["body_paths"]) if p.endswith("/Galbot/base_link"))
        head = next(i for i, p in enumerate(scene["body_paths"]) if p.endswith("/Galbot/head_link2"))

        def frame(index):
            phase = str(scene["phases"][index])
            poses = scene["poses"][index]
            bx, cy, _ = poses[base, :3]
            # Fixed overview and a synchronized virtual camera near the head.
            renderer.camera_eye = [-3.6, -4.8, 3.2]
            renderer.camera_target = [0.85, -0.15, 1.05]
            renderer.fov = 57.0
            renderer.set_camera()
            overview = renderer.render(poses)
            eye_height = float(poses[head, 2]) + 0.38
            renderer.camera_eye = [float(bx) + 0.10, float(cy), eye_height]
            if phase.startswith(("drive_", "exit_")) or phase in ("settle", "done"):
                renderer.camera_target = [1.35, float(cy), max(0.97, eye_height - 0.35)]
                renderer.fov = 88.0
            elif cy < -0.8:
                renderer.camera_target = [0.85, -1.55 if "checkout" in phase or "loan" in phase else -2.10, 0.97]
                renderer.fov = 78.0
            else:
                renderer.camera_target = [1.35, 1.59 if cy > 1 else 0.0, 1.63 if cy > 1 else 1.00]
                renderer.fov = 70.0
            renderer.set_camera()
            robot_view = renderer.render(poses)
            canvas = np.concatenate((overview, robot_view), axis=1)
            canvas[:, width // 2 - 2:width // 2 + 2] = (28, 32, 36)
            return canvas
        selected = ("settle", "lift_return", "blocker_park", "return_insert", "release_return", "release_blocker", "blocker_withdraw", "loan_extract", "release_loan", "raise_for_shelf_b", "stow_return", "lower_for_travel", "done")
        selected = tuple(dict.fromkeys((*selected, str(scene["phases"][-1]))))
        for phase in selected:
            indices = np.flatnonzero(scene["phases"] == phase)
            if len(indices):
                Image.fromarray(frame(int(indices[-1]))).save(directory / (phase + ".png"))
        if args.preview_only:
            return
        output = directory / "s1-library.mp4"
        writer = imageio_ffmpeg.write_frames(str(output), (width, height), fps=fps, codec="libx264", quality=8,
            pix_fmt_in="rgb24", pix_fmt_out="yuv420p", macro_block_size=1, ffmpeg_log_level="error",
            output_params=["-movflags", "+faststart"])
        writer.send(None)
        try:
            for i in range(len(scene["poses"])):
                writer.send(frame(i))
                if i % 300 == 0:
                    print(f"Rendered {i+1}/{len(scene['poses'])}", flush=True)
        finally:
            writer.close()
        metadata = dict(frames=len(scene["poses"]), fps=fps, duration_seconds=len(scene["poses"]) / fps,
                        width=width, height=height, simulation_success=report["success"], speedup=args.speedup,
                        renderer="USD geometry and measured CPU PhysX poses; CUDA rasterization",
                        lighting="matte diffuse; shadow maps and specular reflection disabled",
                        views=["fixed overview", "virtual camera near robot head; task-directed aim"], labels=False)
        output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(json.dumps(metadata), flush=True)


if __name__ == "__main__":
    main()
