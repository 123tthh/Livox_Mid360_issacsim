"""Static contracts for the self-built fixed G1-5010 object-field scenes."""

from __future__ import annotations

import unittest
from pathlib import Path

from PIL import Image


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
        self.assertIn("custom int mid360Validation:objectCount = 10", payload)
        self.assertIn("custom bool mid360Validation:allObjectsTouchGround = 1", payload)
        self.assertIn("custom double mid360Validation:projectionWallDistanceM = 4.75", payload)
        self.assertIn("custom bool mid360Validation:angularProjectionTarget = 1", payload)
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
            "ProjectionWall",
        ):
            self.assertIn(f'"{name}"', payload)

    def test_profile_specific_rviz_accumulation(self) -> None:
        petal = (ROOT / "config/mid360_object_field_petal_4s.rviz").read_text()
        rotary = (ROOT / "config/mid360_object_field_rotary_0p1s.rviz").read_text()
        launcher = (ROOT / "scripts/run_rviz2_mid360.sh").read_text()
        self.assertIn("Decay Time: 4.2", petal)
        self.assertIn("MID360 Petal - full 4.2 s pattern", petal)
        self.assertIn("Decay Time: 0.1", rotary)
        self.assertIn("MID360 Rotary - one 0.1 s revolution", rotary)
        self.assertIn("mid360_object_field_petal_4s.rviz", launcher)
        self.assertIn("mid360_object_field_rotary_0p1s.rviz", launcher)

    def test_angular_pattern_comparison_figure(self) -> None:
        figure = ROOT / "docs/validation/MID360_Petal_vs_Rotary_Angular_Pattern.png"
        with Image.open(figure) as image:
            self.assertEqual(image.format, "PNG")
            self.assertEqual(image.size, (1800, 920))
        generator = (ROOT / "scripts/generate_scan_pattern_comparison.py").read_text()
        self.assertIn("load_trajectory(TRAJECTORY)", generator)
        self.assertIn("40 fixed elevation channels", generator)

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
