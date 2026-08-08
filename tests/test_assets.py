"""Repository-level contracts for the published MID360 assets."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from validate_assets import validate  # noqa: E402


class AssetTests(unittest.TestCase):
    def test_catalog_hashes(self) -> None:
        self.assertEqual(len(validate()), 2)

    def test_shared_mid360_contract(self) -> None:
        manifests = [
            json.loads((ROOT / "assets" / directory / "manifest.json").read_text())
            for directory in ("g1_29dof_rev_1_0", "g1_29dof_mode_13_5010")
        ]
        for manifest in manifests:
            sensor = manifest["mid360"]
            self.assertEqual(sensor["emitters"], 40)
            self.assertEqual(sensor["scan_rate_hz"], 10)
            self.assertEqual(sensor["points_per_second"], 200000)
            self.assertEqual(sensor["elevation_deg"], [-7.0, 52.0])
            self.assertEqual(sensor["robot_frame_elevation_deg"], [-52.0, 7.0])
            self.assertEqual(sensor["mount_roll_deg"], 180.0)
        self.assertEqual(manifests[0]["robot"]["wrist_motor"], "4010")
        self.assertEqual(manifests[1]["robot"]["wrist_motor"], "5010")


if __name__ == "__main__":
    unittest.main()
