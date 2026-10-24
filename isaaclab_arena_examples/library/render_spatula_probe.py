# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Render recorded PhysX contact probes without changing the measured motion."""

import argparse
import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation

from PIL import Image, ImageDraw


def main():
    """Save measured phase stills and optionally a real-time two-view video."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--case", type=int, default=0)
    parser.add_argument("--video", action="store_true")
    parser.add_argument(
        "--phase", nargs="+", help="Render only these phase stills; video still uses the complete capture."
    )
    args = parser.parse_args()
    directory = args.capture_dir
    report = json.loads((directory / "result.json").read_text())
    scene = np.load(directory / "scene.npz", allow_pickle=False)
    vertices, triangles = scene["vertices"], scene["triangles"]
    owners, colors = scene["vertex_body_ids"], scene["colors"]
    cy = args.case * report.get("case_spacing_m", 0.65)
    keep = np.abs(vertices[triangles].mean(axis=1)[:, 1] - cy) < 0.26
    # Movable meshes are owner-local; select their bodies separately.
    paths = scene["body_paths"].tolist()
    dynamic = np.array([f"/Case{args.case}/" in p for p in paths])[owners[triangles[:, 0]]]
    static = owners[triangles[:, 0]] == 0
    faces = triangles[(static & keep) | dynamic]
    face_colors = colors[faces].mean(axis=1)
    used, compact_faces = np.unique(faces, return_inverse=True)
    vertices, owners = vertices[used], owners[used]
    faces = compact_faces.reshape(-1, 3)
    robot_renderer = None
    if report.get("drive_mode", "").startswith("S1"):
        from isaaclab_arena_examples.library.render_video import LibraryRenderer

        class MatteRenderer(LibraryRenderer):
            """Render the measured robot without moving shadow artifacts."""

            specular_strength = 0.0

            def visibility(self, world, position, normal):
                return self.torch.ones_like(position[..., :1])

        robot_renderer = MatteRenderer(scene, 640, 646)
        robot_renderer.camera_eye = [1.4, -2.0, 1.9]
        robot_renderer.camera_target = [-0.5, 0, 1.0]
        robot_renderer.fov = 48
        robot_renderer.set_camera()
    centers = [np.array([-0.63, cy - 0.58, 1.38]), np.array([-0.65, cy + 0.02, 1.04])]
    target = np.array([-0.01, cy, 0.98])
    dual = report.get("mode") in ("slide", "extract")
    if dual:
        centers[0] = np.array([-0.85, cy - 0.75, 1.50])
        target = np.array([-0.08, cy - 0.04, 0.98])
    light = np.array([-0.5, -0.3, 1.0])
    light /= np.linalg.norm(light)

    def frame(index):
        poses = scene["poses"][index]
        rotations = Rotation.from_quat(poses[:, 3:]).as_matrix()
        world = np.einsum("nij,nj->ni", rotations[owners], vertices) + poses[owners, :3]
        image = Image.new("RGB", (1280, 720), (239, 240, 235))
        draw = ImageDraw.Draw(image)
        for panel, eye in enumerate(centers):
            if panel == 0 and robot_renderer is not None:
                image.paste(Image.fromarray(robot_renderer.render(poses)), (0, 44))
                draw.text((20, 56), "S1 joint-driven tool / measured poses", fill=(20, 25, 30))
                continue
            panel_image = Image.new("RGB", (640, 646), (239, 240, 235))
            panel_draw = ImageDraw.Draw(panel_image)
            view_target = target
            if dual and panel == 1:
                name = "ReturnBook" if str(scene["phases"][index]).startswith("return_") else "Book1"
                book_id = paths.index(f"/World/Case{args.case}/{name}")
                view_target = poses[book_id, :3]
                eye = view_target + [-0.65, 0.02, 0.08]
            forward = view_target - eye
            forward /= np.linalg.norm(forward)
            right = np.cross(forward, [0, 0, 1])
            right /= np.linalg.norm(right)
            up = np.cross(right, forward)
            camera = (world - eye) @ np.stack([right, up, forward]).T
            projected = camera[:, :2] / camera[:, 2:3] * (680 if dual and panel == 0 else 900)
            projected[:, 0] += 320 + 640 * panel
            projected[:, 1] = 390 - projected[:, 1]
            depth = camera[faces, 2].mean(axis=1)
            for fi in np.argsort(depth)[::-1]:
                face = faces[fi]
                if camera[face, 2].min() <= 0:
                    continue
                normal = np.cross(world[face[1]] - world[face[0]], world[face[2]] - world[face[0]])
                normal /= max(np.linalg.norm(normal), 1e-12)
                shade = 0.55 + 0.45 * abs(normal @ light)
                color = tuple(np.clip(face_colors[fi] * shade * 255, 0, 255).astype(int))
                points = [tuple(p - [panel * 640, 44]) for p in projected[face]]
                panel_draw.polygon(points, fill=color)
            image.paste(panel_image, (panel * 640, 44))
            draw.text(
                (20 + panel * 640, 56),
                "oblique view" if panel == 0 else "front / gap view",
                fill=(20, 25, 30),
            )
        case = report["cases"][args.case]
        text = (
            f"PhysX measured poses | initial {case['initial_angle_deg']:+g} deg | "
            f"tool {report['thickness_m'] * 1000:g} mm | "
            f"t={scene['times'][index]:.2f}s | {scene['phases'][index]}"
        )
        draw.rectangle((0, 0, 1280, 44), fill=(30, 37, 45))
        draw.text((20, 15), text, fill=(255, 255, 255))
        draw.line((640, 45, 640, 719), fill=(100, 100, 100), width=2)
        draw.text(
            (20, 698),
            (
                "Red: moved book; blue: return book; green: neighbors. Real-time playback."
                if report.get("return_book")
                else "Red: target; green: dynamic neighbors; gray: force-limited tool. Real-time playback."
            ),
            fill=(20, 25, 30),
        )
        return image

    for phase in dict.fromkeys(scene["phases"].tolist()):
        if args.phase and phase not in args.phase:
            continue
        indices = np.flatnonzero(scene["phases"] == phase)
        frame(int(indices[-1])).save(directory / f"case{args.case}-{phase}.png")
    if args.video:
        import imageio_ffmpeg

        fps = 1 / float(np.median(np.diff(scene["times"])))
        video_name = "s1-library.mp4" if report.get("return_book") else f"case{args.case}-contact.mp4"
        writer = imageio_ffmpeg.write_frames(
            str(directory / video_name),
            (1280, 720),
            fps=fps,
            codec="libx264",
            quality=8,
            macro_block_size=1,
            ffmpeg_log_level="error",
            output_params=["-movflags", "+faststart"],
        )
        writer.send(None)
        for index in range(len(scene["times"])):
            writer.send(np.asarray(frame(index)))
        writer.close()


if __name__ == "__main__":
    main()
