# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Read the S1 glTF MDL base colors and UV textures for offline previews."""


def vertex_colors(prim, mesh, cache):
    """Return triangle-corner colors from the bound MDL materials and UVs.

    Args:
        prim: Visible USD mesh.
        mesh: Triangulated mesh with USD corner metadata.
        cache: Material and decoded texture cache.

    Returns:
        Linear RGB colors matching the expanded mesh vertices.
    """
    import numpy as np
    from PIL import Image
    from pxr import UsdGeom, UsdShade
    from isaaclab_arena_examples.bento.capture import _vertex_colors

    if "usd_corners" not in mesh.metadata:
        return _vertex_colors(prim, mesh, cache)
    faces = np.asarray(mesh.faces).copy()
    indices = faces.reshape(-1)
    normals = np.asarray(mesh.vertex_normals)[indices]
    subsets = UsdShade.MaterialBindingAPI(prim).GetMaterialBindSubsets()
    groups = [(prim, np.ones(len(faces), dtype=bool))]
    groups.extend((s.GetPrim(), np.isin(mesh.metadata["usd_face_ids"], s.GetIndicesAttr().Get())) for s in subsets)
    uv = None
    for name in ("st", "st0"):
        pv = UsdGeom.PrimvarsAPI(prim).GetPrimvar(name)
        values = pv.ComputeFlattened() if pv else None
        if values:
            values = np.asarray(values)
            if pv.GetInterpolation() == UsdGeom.Tokens.faceVarying:
                uv = values[mesh.metadata["usd_corners"].reshape(-1)]
            elif pv.GetInterpolation() in (UsdGeom.Tokens.vertex, UsdGeom.Tokens.varying):
                uv = values[indices]
            break
    colors = np.full((len(indices), 3), (0.48, 0.53, 0.60), dtype=np.float32)
    for source, selected in groups:
        material = UsdShade.MaterialBindingAPI(source).ComputeBoundMaterial()[0]
        if not material:
            continue
        factor, texture = None, None
        for child in material.GetPrim().GetChildren():
            shader = UsdShade.Shader(child)
            if not shader:
                continue
            for name in ("base_color_factor", "diffuseColor", "diffuse_color_constant", "diffuse_tint", "base_color"):
                attr = shader.GetInput(name)
                value = attr.Get() if attr else None
                if value is not None and hasattr(value, "__len__") and len(value) == 3:
                    factor = np.asarray(value, dtype=np.float32)
                    color_input = shader.GetInput("base_color_texture")
                    connections = color_input.GetAttr().GetConnections() if color_input else []
                    if connections:
                        node = UsdShade.Shader(prim.GetStage().GetPrimAtPath(connections[0].GetPrimPath()))
                        asset = node.GetInput("texture").Get()
                        if asset:
                            texture = asset.resolvedPath
                    break
            if factor is not None:
                break
        if factor is None:
            continue
        selected = np.repeat(selected, 3)
        colors[selected] = factor
        if texture and uv is not None:
            key = ("s1_texture", texture)
            if key not in cache:
                with Image.open(texture) as image:
                    srgb = np.asarray(image.convert("RGB"), dtype=np.float32) / 255.0
                cache[key] = np.where(srgb <= 0.04045, srgb / 12.92, ((srgb + 0.055) / 1.055) ** 2.4)
            pixels = cache[key]
            coords = uv[selected] % 1.0
            x = np.clip((coords[:, 0] * (pixels.shape[1] - 1)).astype(int), 0, pixels.shape[1] - 1)
            y = np.clip(((1.0 - coords[:, 1]) * (pixels.shape[0] - 1)).astype(int), 0, pixels.shape[0] - 1)
            colors[selected] *= pixels[y, x]
    mesh.vertices = np.asarray(mesh.vertices)[indices]
    mesh.faces = np.arange(len(indices)).reshape(-1, 3)
    mesh.vertex_normals = normals
    return colors
