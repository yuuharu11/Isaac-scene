# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Render measured S1 library task poses with aisle-wide navigation views."""

import argparse
import json
from pathlib import Path


def camera_track(scene, smooth=True):
    """Return virtual camera poses; measured robot and book poses remain untouched.

    Args:
        scene: Measured pose archive.
        smooth: Apply two causal camera-only low-pass stages with 0.3 s time constants.

    Returns:
        Arrays of eye positions, targets and vertical fields of view.
    """
    import numpy as np

    base = next(i for i, p in enumerate(scene["body_paths"]) if p.endswith("/Galbot/base_link"))
    head = next(i for i, p in enumerate(scene["body_paths"]) if p.endswith("/Galbot/head_link2"))
    batch_two = any(str(path).endswith("/Book_loan_book2") for path in scene["body_paths"])
    return_y, checkout_y, shelf_b_y = (-2.15, -1.50, 1.805) if batch_two else (-2.10, -1.55, 1.59)
    values = []
    for phase, poses in zip(scene["phases"], scene["poses"]):
        bx, cy, _ = poses[base, :3]
        height = float(poses[head, 2]) + 0.38
        if phase == "done" and cy < -0.8:
            target, fov = [0.85, checkout_y, 0.97], 78.0
        elif phase.startswith(("drive_", "exit_")) or phase == "settle":
            target, fov = [1.35, float(cy), max(0.97, height - 0.35)], 88.0
        elif cy < -0.8:
            target = [0.85, checkout_y if "checkout" in phase or "loan" in phase else return_y, 0.97]
            fov = 78.0
        else:
            target, fov = [1.35, shelf_b_y if cy > 1 else 0.0, 1.63 if cy > 1 else 1.00], 70.0
        values.append([float(bx) + 0.10, float(cy), height, *target, fov])
    values = np.asarray(values)
    if smooth:
        times = scene["times"]
        first = values[0].copy()
        second = first.copy()
        for i in range(1, len(values)):
            alpha = -np.expm1(-(times[i] - times[i - 1]) / 0.3)
            first += alpha * (values[i] - first)
            second += alpha * (first - second)
            values[i] = second
    return values[:, :3], values[:, 3:6], values[:, 6]


def main():
    """Render representative stills or encode the completed physical pose archive."""
    import numpy as np

    import imageio_ffmpeg
    from PIL import Image, ImageDraw, ImageFont

    from isaaclab_arena_examples.library.render_video import LibraryRenderer

    class StableLibraryRenderer(LibraryRenderer):
        """Use soft material highlights without moving shadow-map edges."""

        def visibility(self, world, position, normal):
            """Return uniform visibility for the shadow-free presentation lighting."""
            return self.torch.ones_like(position[..., :1])

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--preview-only", action="store_true")
    parser.add_argument("--output-dir", type=Path, help="Write renders separately from the source capture.")
    parser.add_argument("--camera", choices=("smooth", "legacy"), default="smooth")
    parser.add_argument("--start", type=float, default=0, help="First physical time in seconds.")
    parser.add_argument("--end", type=float, help="Exclusive last physical time in seconds.")
    parser.add_argument(
        "--phase", nargs="+", help="Render only the named phase stills; video uses all measured frames."
    )
    parser.add_argument("--speedup", type=float, default=1.5, help="Playback speed relative to recorded physics.")
    parser.add_argument("--crf", type=int, default=24, help="H.264 quality: lower is larger and sharper (0–51).")
    parser.add_argument("--preset", default="slow", choices=("fast", "medium", "slow"))
    args = parser.parse_args()
    assert args.speedup > 0, "Playback speed must be positive."
    assert 0 <= args.crf <= 51, "CRF must be between 0 and 51."
    directory = args.capture_dir
    report = json.loads((directory / "result.json").read_text())
    fps = json.loads((directory / "capture.json").read_text())["sampling_fps"] * args.speedup
    output_dir = args.output_dir or directory
    output_dir.mkdir(parents=True, exist_ok=True)
    expanded_room = report.get("scene", {}).get("room", {}).get("shelves", 2) > 2
    width, height = 1920, 720
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 20)
    except OSError:
        font = ImageFont.load_default()
    with np.load(directory / "scene.npz", allow_pickle=False) as scene:
        measured_poses = scene["poses"]
        measured_phases = scene["phases"]
        renderer = StableLibraryRenderer(scene, width // 2, height)
        eyes, targets, fovs = camera_track(scene, smooth=args.camera == "smooth")
        indices_to_render = np.flatnonzero(
            (scene["times"] >= args.start) & (scene["times"] < (args.end if args.end is not None else float("inf")))
        )
        assert len(indices_to_render), "Requested interval contains no measured frames."

        def frame(index):
            poses = measured_poses[index]
            # Fixed overview and a synchronized virtual camera near the head.
            renderer.camera_eye = [-0.85, -5.5, 2.85] if expanded_room else [-3.6, -4.8, 3.2]
            renderer.camera_target = [-0.45, 0.0, 1.0] if expanded_room else [0.85, -0.15, 1.05]
            renderer.fov = 67.0 if expanded_room else 57.0
            renderer.set_camera()
            overview = renderer.render(poses)
            renderer.camera_eye = eyes[index].tolist()
            renderer.camera_target = targets[index].tolist()
            renderer.fov = float(fovs[index])
            renderer.set_camera()
            robot_view = renderer.render(poses)
            canvas = np.concatenate((overview, robot_view), axis=1)
            canvas[:, width // 2 - 2 : width // 2 + 2] = (28, 32, 36)
            image = Image.fromarray(canvas)
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, width, 38), fill=(24, 36, 40))
            draw.text(
                (18, 6),
                "LIBRARY / SINGLE BOOK" if report.get("batch_books", 1) == 1 else "LIBRARY / TWO BOOKS",
                font=font,
                fill=(235, 237, 228),
            )
            phase = str(measured_phases[index])
            returning = (
                "loan" not in phase
                and "checkout" not in phase
                and "shelf_b" not in phase
                and phase != "lower_for_travel"
                and phase != "done"
            )
            role = (
                (
                    "LEFT: RETURN BOOK  /  RIGHT: CLEAR SHELF"
                    if report.get("batch_books", 1) == 1
                    else "TWO-BOOK RETURN / SHELF CLEARING"
                )
                if returning
                else ("SINGLE-BOOK LOAN" if report.get("batch_books", 1) == 1 else "TWO-BOOK LOAN")
            )
            draw.text((width // 2 + 18, 6), role, font=font, fill=(235, 237, 228))
            draw.rectangle((0, height - 34, width, height), fill=(24, 36, 40))
            draw.text((18, height - 29), phase.replace("_", " ").upper(), font=font, fill=(235, 237, 228))
            draw.text(
                (width // 2 + 18, height - 29),
                f"Physics {float(scene['times'][index]):.1f}s / playback {args.speedup:g}x",
                font=font,
                fill=(235, 237, 228),
            )
            return np.asarray(image)

        selected = (
            "settle",
            "lift_return",
            "blocker_park",
            "return_insert",
            "release_return",
            "release_blocker",
            "blocker_withdraw",
            "loan_extract",
            "release_loan",
            "raise_for_shelf_b",
            "stow_return",
            "lower_for_travel",
            "done",
        )
        selected = tuple(dict.fromkeys((*selected, str(measured_phases[-1]))))
        if args.phase:
            selected = tuple(args.phase)
        for phase in selected:
            indices = np.flatnonzero(measured_phases == phase)
            if len(indices):
                Image.fromarray(frame(int(indices[-1]))).save(output_dir / (phase + ".png"))
        if args.preview_only:
            return
        output = output_dir / "s1-library.mp4"
        writer = imageio_ffmpeg.write_frames(
            str(output),
            (width, height),
            fps=fps,
            codec="libx264",
            quality=None,
            pix_fmt_in="rgb24",
            pix_fmt_out="yuv420p",
            macro_block_size=1,
            ffmpeg_log_level="error",
            output_params=["-crf", str(args.crf), "-preset", args.preset, "-movflags", "+faststart"],
        )
        writer.send(None)
        try:
            for i in indices_to_render:
                writer.send(frame(i))
                if i % 300 == 0:
                    print(f"Rendered {i+1}/{len(measured_poses)}", flush=True)
        finally:
            writer.close()
        metadata = dict(
            frames=len(indices_to_render),
            fps=fps,
            duration_seconds=len(indices_to_render) / fps,
            width=width,
            height=height,
            simulation_success=report["success"],
            speedup=args.speedup,
            encoding=dict(codec="libx264", crf=args.crf, preset=args.preset, pixel_format="yuv420p"),
            renderer="USD geometry and measured CPU PhysX poses; CUDA rasterization",
            lighting="diffuse, broad material reflections and local screen-space ambient occlusion; no shadow maps",
            views=["fixed overview", "virtual camera near robot head; task-directed aim"],
            labels=True,
            camera=args.camera,
            source=str(directory),
            physical_interval_s=[
                float(scene["times"][indices_to_render[0]]),
                float(scene["times"][indices_to_render[-1]]),
            ],
        )
        output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(json.dumps(metadata), flush=True)


if __name__ == "__main__":
    main()
