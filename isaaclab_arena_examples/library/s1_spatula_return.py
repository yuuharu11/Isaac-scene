# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Clear a shelf and return a staged book using S1's contact-driven dual spatula."""

from isaaclab_arena_examples.library.dual_spatula_probe import parse_args, run


def main():
    """Run the shelf-front return workflow with gravity and force-limited jaws."""
    args = parse_args(return_workflow=True)
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

    with SimulationAppContext(args):
        run(args)


if __name__ == "__main__":
    main()
