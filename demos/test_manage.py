# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Check that artifact registration preserves runs and confines filesystem writes."""

import json
import tempfile
import unittest
from pathlib import Path

import manage


class RegistrationSafetyTest(unittest.TestCase):
    def test_failed_run_cannot_replace_latest(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "capture"
            source.mkdir()
            (source / "result.json").write_text(json.dumps({"success": True}))
            (source / "video.mp4").write_bytes(b"test fixture")
            demo = {"id": "sample", "video": "video.mp4"}
            outputs = root / "outputs"
            first = manage.register(outputs, demo, source, "first", promote=True)
            (source / "result.json").write_text(json.dumps({"success": False}))
            with self.assertRaises(AssertionError):
                manage.register(outputs, demo, source, "failed", promote=True)
            self.assertEqual(manage.latest_run(outputs, "sample"), first)
            self.assertFalse((first.parent / "failed").exists())
            with self.assertRaises(AssertionError):
                manage.register(outputs, demo, source, "first")
            self.assertTrue(json.loads((first / "result.json").read_text())["success"])

    def test_promotion_migrates_legacy_selection(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "capture"
            source.mkdir()
            (source / "result.json").write_text(json.dumps({"success": True}))
            (source / "video.mp4").write_bytes(b"test fixture")
            demo = {"id": "sample", "video": "video.mp4"}
            outputs = root / "outputs"
            first = manage.register(outputs, demo, source, "first")
            legacy = first.parent.parent / "latest"
            legacy.symlink_to(Path("runs") / "first", target_is_directory=True)
            self.assertEqual(manage.latest_run(outputs, "sample"), first)
            second = manage.register(outputs, demo, source, "second", promote=True)
            self.assertFalse(legacy.is_symlink())
            self.assertEqual(manage.latest_run(outputs, "sample"), second)
            self.assertTrue(first.is_dir())

    def test_latest_selection_rejects_escaping_runs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            outputs = root / "outputs"
            runs = manage.runs_dir(outputs, "sample")
            runs.mkdir(parents=True)
            outside = root / "outside"
            outside.mkdir()
            (runs / "escape").symlink_to(outside, target_is_directory=True)
            selection = runs.parent / "latest.json"
            for run in ("../outside", "escape"):
                selection.write_text(json.dumps({"schema_version": 1, "run": run}))
                with self.assertRaises(AssertionError):
                    manage.latest_run(outputs, "sample")

    def test_paths_cannot_escape_outputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            outputs = root / "outputs"
            outputs.mkdir()
            (outputs / "demos").symlink_to(root, target_is_directory=True)
            with self.assertRaises(AssertionError):
                manage.runs_dir(outputs, "sample")
            with self.assertRaises(AssertionError):
                manage.identifier("../elsewhere")


if __name__ == "__main__":
    unittest.main()
