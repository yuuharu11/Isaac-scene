# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Read the original Galbot glTF MDL base colors for lightweight pose captures."""

import numpy as np

from isaaclab_arena_examples.bento.capture import _vertex_colors


def _material(prim, cache):
    """Return the bound MDL base factor and optional base-color sampler."""
    from pxr import UsdShade

    material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if not material:
        return None
    key = ("library_mdl", str(material.GetPath()))
    if key in cache:
        return cache[key]
    result = None
    for child in material.GetPrim().GetChildren():
        shader = UsdShade.Shader(child)
        if not shader:
            continue
        factor = shader.GetInput("base_color_factor")
        if not factor or factor.Get() is None:
            continue
        sampler = shader.GetInput("base_color_texture")
        connected = sampler.GetConnectedSource() if sampler else None
        result = (np.asarray(factor.Get(), dtype=np.float32), connected[0] if connected else None)
        break
    cache[key] = result
    return result


def _sample(sampler, uv, cache):
    """Sample an sRGB glTF texture with its authored wrapping and UV transform."""
    from PIL import Image

    def value(name, default):
        item = sampler.GetInput(name)
        result = item.Get() if item else None
        return default if result is None else result

    asset = value("texture", None)
    assert asset and asset.resolvedPath, "MDL base-color texture cannot be resolved."
    key = ("library_texture", asset.resolvedPath)
    if key not in cache:
        srgb = np.asarray(Image.open(asset.resolvedPath).convert("RGB"), dtype=np.float32) / 255
        cache[key] = np.where(srgb <= 0.04045, srgb / 12.92, ((srgb + 0.055) / 1.055) ** 2.4)
    image = cache[key]
    uv = uv * np.asarray(value("scale", (1, 1)))
    angle = float(value("rotation", 0))
    uv = uv @ np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
    uv += np.asarray(value("offset", (0, 0)))
    for axis, name in enumerate(("wrap_s", "wrap_t")):
        wrap = value(name, 10497)
        if wrap == 10497:
            uv[:, axis] %= 1
        elif wrap == 33648:
            uv[:, axis] = 1 - np.abs(uv[:, axis] % 2 - 1)
        else:
            uv[:, axis] = uv[:, axis].clip(0, 1)
    x = uv[:, 0] * (image.shape[1] - 1)
    y = (1 - uv[:, 1]) * (image.shape[0] - 1)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    x1, y1 = np.minimum(x0 + 1, image.shape[1] - 1), np.minimum(y0 + 1, image.shape[0] - 1)
    sx, sy = (x - x0)[:, None], (y - y0)[:, None]
    return (image[y0, x0] * (1 - sx) + image[y0, x1] * sx) * (1 - sy) + (
        image[y1, x0] * (1 - sx) + image[y1, x1] * sx
    ) * sy


def asset_vertex_colors(prim, mesh, cache):
    """Bake MDL factors and base textures at triangle corners, preserving material seams.

    Args:
        prim: USD geometry prim with inherited or subset material bindings.
        mesh: Triangulated capture mesh; corners are split in place when required.
        cache: Material and decoded linear texture cache shared by the capture.

    Returns:
        Linear RGB vertex colors. Fine texture detail between vertices is approximated.
    """
    from pxr import UsdGeom, UsdShade

    material = _material(prim, cache)
    subsets = UsdShade.MaterialBindingAPI(prim).GetMaterialBindSubsets()
    if material is None and not any(_material(s.GetPrim(), cache) for s in subsets):
        return _vertex_colors(prim, mesh, cache)
    faces = np.asarray(mesh.faces)
    source = faces.reshape(-1)
    colors = np.full((len(source), 3), (0.48, 0.53, 0.60), dtype=np.float32)
    uv = None
    primvar = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
    if primvar and primvar.HasValue():
        values = np.asarray(primvar.ComputeFlattened(), dtype=np.float32)
        interpolation = primvar.GetInterpolation()
        if interpolation == UsdGeom.Tokens.faceVarying:
            uv = values[mesh.metadata["usd_corners"].reshape(-1)]
        elif interpolation in (UsdGeom.Tokens.vertex, UsdGeom.Tokens.varying):
            uv = values[source]

    def assign(selected, data):
        if data is None:
            return
        factor, sampler = data
        colors[selected] = factor
        if sampler:
            assert uv is not None, f"Textured MDL mesh has no supported UV coordinates: {prim.GetPath()}"
            colors[selected] *= _sample(sampler, uv[selected].copy(), cache)

    assign(np.ones(len(source), dtype=bool), material)
    for subset in subsets:
        selected = np.repeat(np.isin(mesh.metadata["usd_face_ids"], subset.GetIndicesAttr().Get()), 3)
        assign(selected, _material(subset.GetPrim(), cache))
    normals = np.asarray(mesh.vertex_normals)[source].copy()
    mesh.vertices = np.asarray(mesh.vertices)[source].copy()
    mesh.faces = np.arange(len(source)).reshape(-1, 3)
    mesh.vertex_normals = normals
    return colors
