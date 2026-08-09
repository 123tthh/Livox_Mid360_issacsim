"""Contracts for the portable 6/10/15 cm Isaac Sim test scenes."""

from __future__ import annotations

import hashlib
import json
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENE_ROOT = ROOT / "tests/scenes"
CATALOG = json.loads((SCENE_ROOT / "catalog.json").read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class SceneTests(unittest.TestCase):
    def test_no_control_or_mapping_stack_is_embedded(self) -> None:
        forbidden_tokens = (
            "OmniGraph",
            "ROS2PublishJoint",
            "joint_command",
            "joint_states",
            "policyBackend",
            "slamBackend",
            "FAST_LIO",
            "instinct_onboard",
        )
        for path in SCENE_ROOT.rglob("*.usda"):
            payload = path.read_text(encoding="utf-8")
            for token in forbidden_tokens:
                self.assertNotIn(token, payload, f"{token} must not be embedded in {path}")

    def test_scene_catalog_and_hashes(self) -> None:
        self.assertEqual(CATALOG["schema_version"], 1)
        self.assertEqual(set(CATALOG["scenes"]), {"h06cm", "h10cm", "h15cm"})
        for scene_id, entry in CATALOG["scenes"].items():
            for file_kind in ("stage", "environment"):
                file_entry = entry[file_kind]
                path = ROOT / file_entry["path"]
                self.assertTrue(path.is_file(), f"missing {scene_id} {file_kind}")
                self.assertEqual(sha256(path), file_entry["sha256"])

    def test_environment_contracts(self) -> None:
        for scene_id, entry in CATALOG["scenes"].items():
            payload = (ROOT / entry["environment"]["path"]).read_text(encoding="utf-8")
            self.assertTrue(payload.startswith("#usda 1.0"))
            self.assertNotIn("@", payload, f"{scene_id} environment is not self-contained")
            self.assertIn('defaultPrim = "DirectionalStairs"', payload)
            self.assertIn('custom string stairs:forwardAxis = "+X"', payload)
            self.assertRegex(
                payload,
                rf"custom double stairs:stepHeight = {entry['step_height_m']}(?:0)?(?:\s|$)",
            )
            self.assertIn(f"custom int stairs:numSteps = {entry['num_steps']}", payload)
            self.assertRegex(payload, rf"custom double stairs:width = {entry['width_m']:g}(?:\s|$)")
            self.assertIn('prepend apiSchemas = ["PhysicsCollisionAPI"', payload)
            self.assertIn("uniform bool physics:collisionEnabled = true", payload)
            self.assertIsNone(re.search(r"(?:asset|references|payload|subLayers)\s*=", payload))

    def test_ready_to_open_stages(self) -> None:
        sensor_reference = "../../" + CATALOG["default_sensor"]
        for scene_id, entry in CATALOG["scenes"].items():
            payload = (ROOT / entry["stage"]["path"]).read_text(encoding="utf-8")
            environment_name = Path(entry["environment"]["path"]).name
            self.assertIn('defaultPrim = "World"', payload)
            self.assertIn(f"@./stairs/{environment_name}@", payload)
            self.assertIn(f"@{sensor_reference}@", payload)
            self.assertIn('def PhysicsScene "PhysicsScene"', payload)
            self.assertIn("double3 xformOp:translate = (0, 0, 1.2)", payload)
            self.assertIn("quatf xformOp:orient = (0, 1, 0, 0)", payload)
            self.assertNotRegex(payload, r"@[A-Za-z]:|@/home/|@file:")


if __name__ == "__main__":
    unittest.main()
