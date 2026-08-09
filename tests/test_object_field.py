"""Static contracts for the self-built fixed G1-5010 object-field scenes."""

from __future__ import annotations

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENE_ROOT = ROOT / "tests/scenes/object_field"
ENVIRONMENT = SCENE_ROOT / "MID360_Object_Field.usda"
SCENES = {
    "petal": SCENE_ROOT / "MID360_G1_5010_Petal_Object_Field.usda",
    "rotary": SCENE_ROOT / "MID360_G1_5010_Rotary_Object_Field.usda",
}


class ObjectFieldTests(unittest.TestCase):
    def test_environment_is_self_built_and_self_contained(self) -> None:
        payload = ENVIRONMENT.read_text(encoding="utf-8")
        self.assertIn("custom double mid360Validation:radiusM = 5", payload)
        self.assertIn("custom int mid360Validation:objectCount = 9", payload)
        self.assertIn("custom bool mid360Validation:allObjectsTouchGround = 1", payload)
        self.assertNotIn("@", payload)
        for name in (
            "Box",
            "Sphere",
            "Cylinder",
            "Cone",
            "Capsule",
            "ThinWall",
            "Pyramid",
            "Wedge",
            "Arch",
        ):
            self.assertIn(f'"{name}"', payload)

    def test_petal_and_rotary_use_fixed_5010(self) -> None:
        for profile, path in SCENES.items():
            payload = path.read_text(encoding="utf-8")
            self.assertIn(f'custom string mid360Validation:profile = "{profile}"', payload)
            self.assertIn('custom string mid360Validation:robot = "G1 5010 Mode13"', payload)
            self.assertIn("g1_5010_mode_13", payload)
            self.assertIn("custom int mid360Validation:lockedJointCount = 29", payload)
            self.assertEqual(payload.count("float physics:lowerLimit = 0"), 29)
            self.assertEqual(payload.count("float physics:upperLimit = 0"), 29)
            self.assertIn('def PhysicsFixedJoint "G1WorldAnchor"', payload)
            self.assertIn("double3 xformOp:translate = (0, 0, 0.836273)", payload)
            self.assertIn("@MID360_Object_Field.usda@", payload)
            self.assertNotIn("OmniGraph", payload)
            self.assertNotIn("joint_command", payload)
            self.assertNotIn("FAST_LIO", payload)
            self.assertNotIn("instinct_onboard", payload)

        self.assertIn(
            "assets/petal_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Petal_Scan.usd",
            SCENES["petal"].read_text(encoding="utf-8"),
        )
        self.assertIn(
            "assets/rotary_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Rotary_Scan.usd",
            SCENES["rotary"].read_text(encoding="utf-8"),
        )


if __name__ == "__main__":
    unittest.main()
