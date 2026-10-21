# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Attach official left and right Schunk SVH hands to the S1 wrist flanges."""

import math


def spawn_s1_with_hands(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Spawn S1 and replace its stock grippers with articulated five-finger hands.

    Args:
        prim_path: S1 root path.
        cfg: USD spawn config.
        translation: Chassis position.
        orientation: Chassis orientation in WXYZ order.
        **kwargs: Forwarded USD spawner options.

    Returns:
        The spawned S1 root prim.
    """
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

    root = spawn_from_usd(prim_path, cfg, translation=translation, orientation=orientation, **kwargs)
    stage = root.GetStage()
    # Deactivate the complete stock fingers, their joints and the gripper housing meshes.
    for prim in list(Usd.PrimRange(root)):
        if not prim.IsValid():
            continue
        name = prim.GetName()
        if any(part in name for part in ("active_link", "active_joint", "passive_link", "passive_joint", "gripper_base_link")):
            prim.SetActive(False)
    for side, short in (("left", "l"), ("right", "r")):
        rotation = Gf.Quatf(math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0)
        if side == "right":
            # Mirrored anatomy needs an additional roll: both thumbs up, palms inward.
            rotation = rotation * Gf.Quatf(0.0, 0.0, 0.0, 1.0)
        wrist = stage.GetPrimAtPath(f"{prim_path}/{side}_arm_link7")
        path = f"{prim_path}/{side}_svh"
        hand = stage.DefinePrim(path, "Xform")
        hand.GetReferences().AddReference(
            f"{ISAAC_NUCLEUS_DIR}/Robots/Schunk/svh-flat-{short}/svh-flat-{short}_v2.usd"
        )
        # The SVH fingers extend along +Z; align that axis with the wrist's outward -X.
        mount = Gf.Matrix4d().SetRotate(Gf.Quatd(rotation))
        mount.SetTranslateOnly(Gf.Vec3d(-0.14, 0.0, 0.0))
        world = mount * UsdGeom.XformCache().GetLocalToWorldTransform(wrist)
        xform = UsdGeom.Xformable(hand)
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(world)
        stage.GetPrimAtPath(path + "/root_joint").SetActive(False)
        stage.GetPrimAtPath(path + "/base_link").SetActive(False)
        for part in Usd.PrimRange(hand):
            if part.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI.Apply(part).CreateDisableGravityAttr(True)
        joint = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/joints/{side}_svh_mount")
        joint.CreateBody0Rel().SetTargets([wrist.GetPath()])
        joint.CreateBody1Rel().SetTargets([path + f"/{side}_hand_base_link"])
        joint.CreateLocalPos0Attr(Gf.Vec3f(-0.14, 0.0, 0.0))
        joint.CreateLocalRot0Attr(rotation)
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.0))
    return root
