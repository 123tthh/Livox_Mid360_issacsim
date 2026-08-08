"""Repository-level contracts for the published MID360 assets."""

from __future__ import annotations

import hashlib
import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from validate_assets import validate  # noqa: E402
from apply_mid360_nonrepetitive_profile import (  # noqa: E402
    CHANNEL_COUNT,
    EXPECTED_POINTS,
    EXPECTED_TRAJECTORY_SHA256,
    POINT_INTERVAL_NS,
    POINTS_PER_STATE,
    SCAN_RATE_HZ,
    STATE_COUNT,
    TRAJECTORY,
    load_trajectory,
)


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
            self.assertEqual(sensor["scan_type"], "SOLID_STATE")
            self.assertEqual(sensor["emitters_per_state"], POINTS_PER_STATE)
            self.assertEqual(sensor["rtx_emitter_states"], 1)
            self.assertEqual(sensor["trajectory_states"], STATE_COUNT)
            self.assertEqual(sensor["channels"], CHANNEL_COUNT)
            self.assertEqual(sensor["scan_rate_hz"], SCAN_RATE_HZ)
            self.assertEqual(sensor["report_rate_hz"], SCAN_RATE_HZ)
            self.assertEqual(sensor["points_per_second"], 200000)
            self.assertEqual(sensor["nominal_elevation_deg"], [-7.0, 52.0])
            self.assertEqual(sensor["robot_frame_nominal_elevation_deg"], [-52.0, 7.0])
            self.assertEqual(sensor["trajectory_points"], EXPECTED_POINTS)
            self.assertEqual(sensor["trajectory_duration_s"], 4.0)
            self.assertEqual(sensor["trajectory_sha256"], EXPECTED_TRAJECTORY_SHA256)
            self.assertEqual(sensor["mount_roll_deg"], 180.0)
        self.assertEqual(manifests[0]["robot"]["wrist_motor"], "4010")
        self.assertEqual(manifests[1]["robot"]["wrist_motor"], "5010")

    def test_official_nonrepetitive_trajectory(self) -> None:
        trajectory = load_trajectory(TRAJECTORY)
        self.assertEqual(len(trajectory.azimuth_deg), EXPECTED_POINTS)
        self.assertAlmostEqual(min(trajectory.elevation_deg), -7.2123, places=4)
        self.assertAlmostEqual(max(trajectory.elevation_deg), 52.164, places=4)

        # Each 0.1 s state must differ. A repeated state would recreate the
        # old mechanically repeating scan instead of accumulating coverage.
        state_hashes = []
        for azimuth, elevation in trajectory.states():
            digest = hashlib.sha256()
            digest.update(repr(azimuth).encode("ascii"))
            digest.update(repr(elevation).encode("ascii"))
            state_hashes.append(digest.digest())
        self.assertEqual(len(state_hashes), STATE_COUNT)
        self.assertEqual(len(set(state_hashes)), STATE_COUNT)
        self.assertEqual((POINTS_PER_STATE - 1) * POINT_INTERVAL_NS, 99_995_000)


if __name__ == "__main__":
    unittest.main()
