# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Configure the official Galbot One Charlie for isolated control bring-up."""

LEG_JOINTS = tuple(f"leg_joint{i}" for i in range(1, 5))
LEFT_ARM_JOINTS = tuple(f"left_arm_joint{i}" for i in range(1, 8))
RIGHT_ARM_JOINTS = tuple(f"right_arm_joint{i}" for i in range(1, 8))
TCP_BODIES = {"left": "left_gripper_tcp_link", "right": "right_suction_cup_tcp_link"}
TORSO_BODY = "torso_base_link"


def make_galbot_cfg(prim_path="/World/Galbot", position=(0.0, 0.0, 0.0)):
    """Copy the installed official asset configuration with finite bring-up drive gains.

    Args:
        prim_path: Stage path for the robot articulation.
        position: Fixed chassis origin in world coordinates.

    Returns:
        A Galbot articulation config; gravity and self collisions are disabled for this control bench.
    """
    import isaaclab.sim as sim_utils
    from isaaclab_assets.robots.galbot import GALBOT_ONE_CHARLIE_CFG

    cfg = GALBOT_ONE_CHARLIE_CFG.copy()
    cfg.prim_path = prim_path
    cfg.init_state.pos = position
    cfg.spawn.articulation_props = sim_utils.ArticulationRootPropertiesCfg(
        fix_root_link=True,
        enabled_self_collisions=False,
        solver_position_iteration_count=32,
        solver_velocity_iteration_count=8,
    )
    cfg.actuators["leg"].stiffness = {"leg_joint1": 20000.0, "leg_joint[2-4]": 2000.0}
    cfg.actuators["leg"].damping = {"leg_joint1": 1000.0, "leg_joint[2-4]": 100.0}
    for group in ("left_arm", "right_arm", "head"):
        cfg.actuators[group].stiffness = 500.0
        cfg.actuators[group].damping = 40.0
    cfg.actuators["left_gripper"].stiffness = 1000.0
    cfg.actuators["left_gripper"].damping = 50.0
    return cfg


def smooth_fraction(fraction):
    """Return a quintic interpolation weight with zero endpoint velocity and acceleration.

    Args:
        fraction: Elapsed fraction of a motion segment, clamped to [0, 1].

    Returns:
        Smooth interpolation weight.
    """
    value = max(0.0, min(1.0, fraction))
    return value * value * value * (10.0 + value * (-15.0 + 6.0 * value))
