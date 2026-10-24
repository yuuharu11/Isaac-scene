# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Collect continuous library motion and contact evidence without changing physics."""


def read_contact_materials(stage):
    """Return effective material coefficients and PhysX combine modes without editing the stage.

    Args:
        stage: Existing physical USD stage.

    Returns:
        Material-path dictionary with values and authored/default provenance.
    """
    from pxr import PhysxSchema, Usd, UsdPhysics

    # Isolated anonymous schema prim supplies fallback values even when the source
    # material has no authored PhysX API. No prim or attribute is added to stage.
    defaults = Usd.Stage.CreateInMemory()
    default_prim = defaults.DefinePrim("/Material", "Material")
    default_usd = UsdPhysics.MaterialAPI.Apply(default_prim)
    default_physx = PhysxSchema.PhysxMaterialAPI.Apply(default_prim)
    result = {}
    for prim in stage.TraverseAll():
        if not prim.HasAPI(UsdPhysics.MaterialAPI):
            continue
        material = UsdPhysics.MaterialAPI(prim)
        physx = PhysxSchema.PhysxMaterialAPI(prim)
        fields = {
            "static_friction": (material.GetStaticFrictionAttr(), default_usd.GetStaticFrictionAttr()),
            "dynamic_friction": (material.GetDynamicFrictionAttr(), default_usd.GetDynamicFrictionAttr()),
            "restitution": (material.GetRestitutionAttr(), default_usd.GetRestitutionAttr()),
            "friction_combine_mode": (
                physx.GetFrictionCombineModeAttr(),
                default_physx.GetFrictionCombineModeAttr(),
            ),
            "restitution_combine_mode": (
                physx.GetRestitutionCombineModeAttr(),
                default_physx.GetRestitutionCombineModeAttr(),
            ),
        }
        entry = {"physx_material_api_applied": bool(prim.HasAPI(PhysxSchema.PhysxMaterialAPI))}
        authored = {}
        for name, (attribute, fallback) in fields.items():
            value = attribute.Get() if attribute else None
            value = fallback.Get() if value is None else value
            entry[name] = float(value) if isinstance(value, (int, float)) else str(value) if value is not None else None
            authored[name] = bool(attribute and attribute.HasAuthoredValueOpinion())
        entry["authored"] = authored
        result[str(prim.GetPath())] = entry
    return result


def create_contact_diagnostics(dt):
    """Return a PhysX subscription, mutable step clock, and collider-pair summaries.

    Args:
        dt: Physics timestep in seconds.

    Returns:
        Contact subscription, one-element step clock, and JSON-compatible pair dictionary.
    """
    import numpy as np

    import omni.physx
    from pxr import PhysicsSchemaTools

    clock, pairs = [0], {}

    def record(headers, data):
        for header in headers:
            actors = [str(PhysicsSchemaTools.intToSdfPath(x)) for x in (header.actor0, header.actor1)]
            if not any(
                "/Tool" in actor
                or "/Galbot/" in actor
                or actor.startswith("/World/Book_")
                or ("/Library/" in actor and "Neighbor" in actor)
                for actor in actors
            ):
                continue
            colliders = [str(PhysicsSchemaTools.intToSdfPath(x)) for x in (header.collider0, header.collider1)]
            # Sorting the key keeps reversed report headers in a single physical pair.
            key = " | ".join(sorted(colliders))
            entry = pairs.setdefault(
                key,
                dict(actors=actors, colliders=colliders, peak_impulse_Ns=0.0, max_penetration_m=0.0, contacts=0),
            )
            for index in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                contact = data[index]
                impulse = float(np.linalg.norm(list(contact.impulse)))
                penetration = max(0.0, -float(contact.separation))
                entry["contacts"] += 1
                entry["last_time_s"] = float(clock[0] * dt)
                entry.setdefault("first_time_s", float(clock[0] * dt))
                if penetration > entry["max_penetration_m"]:
                    entry.update(max_penetration_m=penetration, penetration_peak_time_s=float(clock[0] * dt))
                if impulse > entry["peak_impulse_Ns"]:
                    entry.update(
                        peak_impulse_Ns=impulse,
                        peak_time_s=float(clock[0] * dt),
                        peak_position_m=[float(x) for x in contact.position],
                        peak_separation_m=float(contact.separation),
                    )

    subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(record)
    return subscription, clock, pairs


class LibraryMotionAudit:
    """Track every physics step, including inactive-arm grasps and neighboring books."""

    def __init__(self, books, tools, catalog, dt, diagnostic_phase=None):
        """Initialize the audit after bodies are initialized and before simulation steps.

        Args:
            books: Mapping of all target and dynamic neighboring rigid objects.
            tools: Side-keyed tool state with articulation, held name, and two contact sensors.
            catalog: Ordered target-book metadata, matching contact-sensor filter order.
            dt: Physics timestep in seconds.
            diagnostic_phase: Optional phase receiving full-rate diagnostic state samples.
        """
        self.books, self.tools, self.catalog, self.dt = books, tools, catalog, dt
        self.initial = {name: self._pose(body) for name, body in books.items() if "Neighbor" in name}
        self.neighbors = {name: dict(max_displacement_m=0.0, max_rotation_deg=0.0) for name in self.initial}
        self.targets = {name: dict(held_samples=0, bilateral_contact_samples=0) for name in catalog}
        self.force_stats, self.references, self.previous_forces = {}, {}, {}
        self.samples = 0
        self.diagnostic_phase, self.diagnostic_samples = diagnostic_phase, []
        self.phase_measurements = {}
        self.force_by_phase = {}

    @staticmethod
    def _pose(body):
        return body.data.root_link_pose_w.torch[0].numpy().copy()

    @property
    def max_neighbor_displacement_m(self):
        """Return the unrounded maximum dynamic-neighbor displacement over all updates."""
        return max((entry["max_displacement_m"] for entry in self.neighbors.values()), default=0.0)

    def update(self, step, phase):
        """Record poses and already-updated sensors after one physical timestep.

        Args:
            step: Current physical step number.
            phase: Current plan phase name or phase dictionary.
        """
        import numpy as np
        from scipy.spatial.transform import Rotation

        phase = phase["name"] if isinstance(phase, dict) else str(phase)
        event = dict(time_s=float(step * self.dt), phase=phase)
        if phase == self.diagnostic_phase:
            self._sample_diagnostic(event)
        self.samples += 1
        for name, initial in self.initial.items():
            measured = self._pose(self.books[name])
            displacement = float(np.linalg.norm(measured[:3] - initial[:3]))
            rotation = float(
                np.degrees((Rotation.from_quat(initial[3:]).inv() * Rotation.from_quat(measured[3:])).magnitude())
            )
            entry = self.neighbors[name]
            for metric, value in (("displacement_m", displacement), ("rotation_deg", rotation)):
                if value > entry["max_" + metric]:
                    entry["max_" + metric] = value
                    entry[metric + "_peak"] = event.copy()
        for name, metadata in self.catalog.items():
            if phase not in (metadata.get("lift_phase"), metadata.get("grasp_phase")):
                continue
            measured = self._pose(self.books[name])
            bottom = float(measured[2] - np.abs(Rotation.from_quat(measured[3:]).as_matrix()[2]) @ [0.08, 0.0175, 0.11])
            entry = self.phase_measurements.setdefault(phase, {}).setdefault(name, {})
            entry.update(final_bottom_m=bottom, final_center_z_m=float(measured[2]), **event)
            entry["max_bottom_m"] = max(bottom, entry.get("max_bottom_m", bottom))
        for side, tool in self.tools.items():
            matrices = [sensor.data.force_matrix_w.torch[0, 0].numpy() for sensor in tool["sensors"]]
            forces = np.stack([np.linalg.norm(matrix, axis=-1) for matrix in matrices], axis=-1)
            assert forces.shape == (len(self.catalog), 2), "Audit sensors must follow catalog target order."
            for index, name in enumerate(self.catalog):
                key = side + "/" + name
                pair = forces[index]
                stats = self.force_stats.setdefault(
                    key, dict(peak_jaw_force_N=[0.0, 0.0], peak_jaw_slew_N_s=[0.0, 0.0])
                )
                previous = self.previous_forces.get(key, pair)
                slew = np.abs(pair - previous) / self.dt
                phase_stats = self.force_by_phase.setdefault(phase, {}).setdefault(
                    key, dict(peak_jaw_force_N=[0.0, 0.0], peak_jaw_slew_N_s=[0.0, 0.0], samples=0)
                )
                phase_stats["samples"] += 1
                phase_stats.setdefault("start_time_s", event["time_s"])
                phase_stats["end_time_s"] = event["time_s"]
                # The first sample of a new phase includes its boundary slew from the previous step.
                for destination in (stats, phase_stats):
                    for jaw in range(2):
                        if pair[jaw] > destination["peak_jaw_force_N"][jaw]:
                            destination["peak_jaw_force_N"][jaw] = float(pair[jaw])
                            destination.setdefault("force_peaks", {})[str(jaw)] = event.copy()
                        if slew[jaw] > destination["peak_jaw_slew_N_s"][jaw]:
                            destination["peak_jaw_slew_N_s"][jaw] = float(slew[jaw])
                            destination.setdefault("slew_peaks", {})[str(jaw)] = event.copy()
                self.previous_forces[key] = pair.copy()
                if tool["held"] != name:
                    self.references.pop(key, None)
                    continue
                entry = self.targets[name]
                intervals = entry.setdefault("held_intervals", [])
                if key not in self.references:
                    intervals.append(dict(side=side, start_time_s=event["time_s"], end_time_s=event["time_s"]))
                intervals[-1]["end_time_s"] = event["time_s"]
                entry["held_samples"] += 1
                entry["bilateral_contact_samples"] += int(bool((pair > 0.05).all()))
                book = self._pose(self.books[name])
                actual = self._pose(tool["tool"])
                relative = Rotation.from_quat(actual[3:]).inv().apply(book[:3] - actual[:3])
                reference = self.references.setdefault(key, relative.copy())
                slip = float(np.linalg.norm(relative - reference))
                if slip >= entry.get("max_relative_slip_m", 0.0):
                    entry.update(max_relative_slip_m=slip, slip_peak=event.copy())
                bottom = float(book[2] - np.abs(Rotation.from_quat(book[3:]).as_matrix()[2]) @ [0.08, 0.0175, 0.11])
                if bottom < entry.get("min_held_bottom_m", float("inf")):
                    entry.update(min_held_bottom_m=bottom, held_bottom_minimum=event.copy())
                withdrawn = self.catalog[name].get("withdraw_phase")
                if withdrawn and phase == withdrawn:
                    entry["withdraw_time_s"] = event["time_s"]
            # Retain contact evidence after withdrawal even when no book is marked held.
            for index, name in enumerate(self.catalog):
                entry = self.targets[name]
                if phase == self.catalog[name].get("withdraw_phase"):
                    entry["withdraw_time_s"] = event["time_s"]
                when = entry.get("withdraw_time_s")
                if when is not None and event["time_s"] > when and float(forces[index].max()) > 0.05:
                    entry["post_withdraw_contact_samples"] = entry.get("post_withdraw_contact_samples", 0) + 1
                    entry["last_post_withdraw_contact"] = dict(side=side, **event)

    def _sample_diagnostic(self, event):
        """Retain measured poses and solver velocities for a selected diagnostic phase.

        Args:
            event: Physical sample timestamp and phase name.
        """
        self.diagnostic_samples.append(
            dict(
                **event,
                books={
                    name: dict(
                        pose=self._pose(self.books[name]).tolist(),
                        velocity=self.books[name].data.root_com_vel_w.torch[0].tolist(),
                    )
                    for name in self.catalog
                },
                tools={
                    side: dict(
                        pose=self._pose(tool["tool"]).tolist(),
                        forces=[sensor.data.force_matrix_w.torch[0, 0].tolist() for sensor in tool["sensors"]],
                    )
                    for side, tool in self.tools.items()
                },
            )
        )

    def summary(self):
        """Return JSON-compatible continuous measurements and their sampling rate."""
        import copy

        targets = copy.deepcopy(self.targets)
        for entry in targets.values():
            count = entry["held_samples"]
            entry["both_jaws_contact_fraction"] = entry["bilateral_contact_samples"] / count if count else 0.0
        return dict(
            sampling_dt_s=float(self.dt),
            samples=self.samples,
            max_neighbor_displacement_m=self.max_neighbor_displacement_m,
            neighbors={
                name: dict(
                    entry,
                    initial_pose=self.initial[name].tolist(),
                    final_pose=self._pose(self.books[name]).tolist(),
                    final_velocity=self.books[name].data.root_com_vel_w.torch[0].tolist(),
                )
                for name, entry in self.neighbors.items()
            },
            books=targets,
            forces=copy.deepcopy(self.force_stats),
            force_by_phase=copy.deepcopy(self.force_by_phase),
            phase_measurements=copy.deepcopy(self.phase_measurements),
        )
