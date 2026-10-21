# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Configure the official Galbot S1 while preserving the One Charlie implementation."""

import os
from pathlib import Path

from isaaclab_arena_examples.library.galbot_robot import (
    LEFT_ARM_JOINTS as LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS as RIGHT_ARM_JOINTS,
    smooth_fraction as smooth_fraction,
)

LIFT_JOINTS = ("torso_lift_joint1",)
TORSO_BODY = "torso_lift_link1"
TCP_BODIES = {"left": "left_arm_link7", "right": "right_arm_link7"}
FRONT_ARM_POSE = {'left_arm_joint1': -0.08755701470539962,
 'left_arm_joint2': -0.750116506170448,
 'left_arm_joint3': 0.549313785314499,
 'left_arm_joint4': -2.190592534644532,
 'left_arm_joint5': 1.6401735147486693,
 'left_arm_joint6': 0.05220827649535593,
 'left_arm_joint7': 1.2851810786825395,
 'right_arm_joint1': 0.08752739968281377,
 'right_arm_joint2': 0.7501162645607984,
 'right_arm_joint3': -0.5493173889774887,
 'right_arm_joint4': 2.1906054353457693,
 'right_arm_joint5': -1.6402031192594855,
 'right_arm_joint6': -0.05219711576469188,
 'right_arm_joint7': -1.2851897236541439}

ASSET_REVISION = "cb758e557a880c70260c7883207334aed2aa3c65"


def make_galbot_cfg(prim_path="/World/Galbot", position=(0.0, 0.0, 0.0)):
    """Return a fixed-chassis S1 config with gravity and self collisions disabled.

    Args:
        prim_path: Robot articulation path.
        position: Chassis origin in metres.

    Returns:
        Configuration using the externally downloaded official USD.
    """
    from isaaclab_arena_examples.library.galbot_s1_hands import spawn_s1_with_hands

    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg

    asset = Path(os.environ.get("GALBOT_S1_USD", "/work/data/galbot_s1_description/usd/galbot_s1.usda"))
    assert asset.is_file(), f"Download the official S1 assets first: {asset}"
    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            func=spawn_s1_with_hands, usd_path=str(asset), variants={"Physics": "physx"},
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                fix_root_link=True, enabled_self_collisions=False,
                solver_position_iteration_count=32, solver_velocity_iteration_count=8,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=position, joint_pos={"torso_lift_joint1": 0.25, **FRONT_ARM_POSE}),
        actuators={
            "lift": ImplicitActuatorCfg(joint_names_expr=["torso_lift_joint1"], stiffness=600000.0, damping=10000.0),
            "arms": ImplicitActuatorCfg(joint_names_expr=[".*_arm_joint.*"], stiffness=10000.0, damping=500.0),
            "head": ImplicitActuatorCfg(joint_names_expr=["head_joint.*"], stiffness=500.0, damping=40.0),
            "hands": ImplicitActuatorCfg(joint_names_expr=["(Left|Right)_Hand_.*"], stiffness=20.0, damping=1.0),
            "wheels": ImplicitActuatorCfg(joint_names_expr=["wheel_.*"], stiffness=1000.0, damping=50.0),
        },
    )
