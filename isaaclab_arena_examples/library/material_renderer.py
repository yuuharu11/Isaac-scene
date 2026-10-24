# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Library-only presentation materials for recorded geometry without UV coordinates."""

from isaaclab_arena_examples.bento.render_video import CudaRenderer


class LibraryMaterialRenderer(CudaRenderer):
    """Preserve captured geometry while assigning restrained presentation materials."""

    def __init__(self, scene, width, height):
        super().__init__(scene, width, height)
        torch = self.torch
        # Channels are specular strength, wood mask, and reserved material data.
        self.materials = torch.zeros_like(self.colors)
        self.materials[:, 0] = 0.018
        wood = (self.colors - torch.tensor([0.38, 0.24, 0.13], device="cuda")).abs().amax(-1) < 0.005
        self.materials[wood, 1] = 1
        for index, path in enumerate(scene["body_paths"].tolist()):
            selected = self.body_ids == index
            if "/Galbot/" in path:
                self.materials[selected, 0] = 0.055
            elif path.startswith(("/World/ToolLeft/", "/World/ToolRight/")):
                self.materials[selected, 0] = 0.24
            elif path.startswith("/World/Book_") or "Neighbor" in path:
                # Keep captured cover and paper colors, including future catalog books.
                self.materials[selected, 0] = 0.012

    def ambient_visibility(self, position, normal, covered):
        """Estimate restrained local occlusion from visible measured geometry."""
        torch = self.torch
        occlusion = torch.zeros_like(position[..., :1])
        # World-space distance and surface orientation reject distant silhouette halos.
        for radius in (3, 7, 13):
            for dx, dy in ((radius, 0), (-radius, 0), (0, radius), (0, -radius)):
                nearby = torch.roll(position, shifts=(dy, dx), dims=(1, 2))
                valid = torch.roll(covered, shifts=(dy, dx), dims=(1, 2)).clone()
                if dy > 0:
                    valid[:, :dy] = False
                elif dy < 0:
                    valid[:, dy:] = False
                if dx > 0:
                    valid[:, :, :dx] = False
                elif dx < 0:
                    valid[:, :, dx:] = False
                delta = nearby - position
                distance = delta.norm(dim=-1, keepdim=True)
                elevation = (normal * delta).sum(-1, keepdim=True) / distance.clamp_min(1e-5)
                weight = (1 - distance / 0.09).clamp(0, 1)
                occlusion += valid * weight * ((elevation - 0.08) / 0.7).clamp(0, 1) / 12
        occlusion = torch.nn.functional.avg_pool2d(
            occlusion.permute(0, 3, 1, 2), kernel_size=3, stride=1, padding=1
        ).permute(0, 2, 3, 1)
        return 1 - 0.38 * occlusion

    def render(self, poses):
        """Return one uint8 RGB image for recorded world poses.

        Args:
            poses: Body poses in world XYZ and XYZW quaternion order.

        Returns:
            Image array in top-to-bottom row order.
        """
        torch, dr = self.torch, self.dr
        with torch.no_grad():
            body_poses = torch.as_tensor(poses, dtype=torch.float32, device="cuda")[self.body_ids]
            quaternions = torch.nn.functional.normalize(body_poses[:, 3:], dim=-1)
            world = self.rotate(self.vertices, quaternions) + body_poses[:, :3]
            normals = self.rotate(self.normals, quaternions)
            camera = (world - self.eye) @ self.view_rotation.T
            near, far = 0.03, 20.0
            clip = (
                torch.stack(
                    (
                        camera[:, 0] * self.focal / (self.width / self.height),
                        camera[:, 1] * self.focal,
                        -(far + near) / (far - near) * camera[:, 2] - 2 * far * near / (far - near),
                        -camera[:, 2],
                    ),
                    dim=-1,
                )
                .unsqueeze(0)
                .contiguous()
            )
            rast, _ = dr.rasterize(self.ctx, clip, self.triangles, resolution=[self.height, self.width])
            attributes = (
                torch.cat((self.colors, normals, world, self.vertices, self.materials), dim=-1)
                .unsqueeze(0)
                .contiguous()
            )
            interpolated, _ = dr.interpolate(attributes, rast, self.triangles)
            color, normal, position = interpolated[..., :3], interpolated[..., 3:6], interpolated[..., 6:9]
            local = interpolated[..., 9:12]
            material = interpolated[..., 12:15]
            # Low-frequency grain follows each face in captured local coordinates.
            face = normal.abs().argmax(dim=-1, keepdim=True)
            across = torch.where(face == 2, local[..., 0:1], local[..., 2:3])
            along = torch.where(face == 2, local[..., 1:2], local[..., 0:1])
            warp = 2.1 * torch.sin(across * 17 + along * 2.7) + 0.8 * torch.sin(along * 13)
            grain = 1 + material[..., 1:2] * (
                0.012 * torch.sin(across * 181 + warp)
                + 0.006 * torch.sin(across * 389 + warp * 1.7)
                + 0.004 * torch.sin(across * 31 + along * 7)
            )
            color = color * grain
            normal = torch.nn.functional.normalize(normal, dim=-1)
            view = torch.nn.functional.normalize(self.eye - position, dim=-1)
            # Shade both sides: USD assets may contain open meshes with mixed winding.
            normal = torch.where((normal * view).sum(-1, keepdim=True) < 0, -normal, normal)
            diffuse = (normal * self.light).sum(-1, keepdim=True).clamp(0, 1)
            visibility = self.visibility(world, position, normal)
            halfway = torch.nn.functional.normalize(view + self.light, dim=-1)
            highlight = (normal * halfway).sum(-1, keepdim=True).clamp(0, 1)
            specular = (highlight.pow(32) + 0.3 * highlight.pow(8)) * material[..., 0:1]
            # A broad environment reflection reveals curved dark parts without recoloring them.
            fresnel = (1 - (normal * view).sum(-1, keepdim=True).clamp(0, 1)).pow(4)
            reflection = material[..., 0:1] * (0.025 + 0.15 * fresnel)
            ambient_visibility = self.ambient_visibility(position, normal, rast[..., 3:4] > 0)
            sky = 0.46 + 0.08 * normal[..., 2:3].clamp(0, 1)
            shaded = (color * (sky + 0.52 * diffuse * visibility) + reflection) * ambient_visibility
            shaded += specular * visibility
            output = torch.where(rast[..., 3:4] > 0, shaded, self.background).contiguous()
            output = dr.antialias(output, rast, clip, self.triangles)
            # USD diffuse colors are linear; export standard sRGB for PNG and MP4.
            linear = output.clamp(0, 1)
            srgb = torch.where(linear <= 0.0031308, linear * 12.92, 1.055 * linear.pow(1 / 2.4) - 0.055)
            return (srgb[0].flip(0) * 255).round().to(torch.uint8).cpu().numpy()
