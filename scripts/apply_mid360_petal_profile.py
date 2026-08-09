#!/usr/bin/env python3
"""Embed the Livox MID-360 non-repetitive firing pattern in bundled USDs.

Run with Isaac Sim 5.1's Python environment (or another Python environment
that provides ``pxr``).  The source trajectory is the 800,000-ray MID-360
table published by Livox-SDK/livox_laser_simulation.  Forty emitter states
represent consecutive 0.1 s hardware frames, so every state contains 20,000
rays and the full table represents four seconds of non-repeating motion.
"""

from __future__ import annotations

import argparse
import array
import csv
import hashlib
import os
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY = ROOT / "assets/common/mid360_official_pattern/mid360.csv"
ASSETS = (
    ROOT / "assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd",
    ROOT / "assets/petal_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Petal_Scan.usd",
    ROOT / "assets/petal_scan/g1_5010_mode_15/Unitree_G1_5010_Mode15_MID360_Petal_Scan.usd",
)
POINT_RATE_HZ = 200_000
SCAN_RATE_HZ = 10
POINTS_PER_STATE = POINT_RATE_HZ // SCAN_RATE_HZ
STATE_COUNT = 40
EMBEDDED_EMITTER_STATE_COUNT = 1
POINT_INTERVAL_NS = 1_000_000_000 // POINT_RATE_HZ
CHANNEL_COUNT = POINTS_PER_STATE
EXPECTED_POINTS = POINTS_PER_STATE * STATE_COUNT
EXPECTED_TRAJECTORY_SHA256 = "aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a"
LIDAR_SUFFIX = "/torso_link/mid360_link/mid360_native_approx"
PATTERN_DATA_NAME = "mid360_nonrepetitive_pattern"
PATTERN_ENCODING = "zlib+u16le:azimuth_centideg,elevation_centideg_plus_1000"


@dataclass(frozen=True)
class Trajectory:
    azimuth_deg: tuple[float, ...]
    elevation_deg: tuple[float, ...]

    def states(self) -> Iterable[tuple[tuple[float, ...], tuple[float, ...]]]:
        for start in range(0, len(self.azimuth_deg), POINTS_PER_STATE):
            stop = start + POINTS_PER_STATE
            yield self.azimuth_deg[start:stop], self.elevation_deg[start:stop]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_trajectory(path: Path = TRAJECTORY) -> Trajectory:
    """Load and validate Livox's azimuth/zenith table.

    The first column is a monotonically increasing sample index despite its
    upstream ``Time/s`` heading.  Timing is reconstructed from the official
    200,000 points/s specification instead of interpreting that heading.
    """

    actual_sha = sha256(path)
    if actual_sha != EXPECTED_TRAJECTORY_SHA256:
        raise ValueError(f"unexpected MID-360 trajectory sha256: {actual_sha}")

    azimuth: list[float] = []
    elevation: list[float] = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = csv.reader(stream)
        header = next(rows, None)
        if header != ["Time/s", "Azimuth/deg", "Zenith/deg"]:
            raise ValueError(f"unexpected MID-360 trajectory header: {header!r}")
        for expected_index, row in enumerate(rows, start=1):
            if len(row) != 3 or int(row[0]) != expected_index:
                raise ValueError(f"invalid trajectory row {expected_index}: {row!r}")
            azimuth_deg = float(row[1]) % 360.0
            elevation_deg = 90.0 - float(row[2])
            if not 0.0 <= azimuth_deg < 360.0:
                raise ValueError(f"azimuth outside 360-degree FOV at row {expected_index}")
            # Livox's reference trace has sub-degree edge excursions around
            # the nominal [-7, 52] degree FOV.  Reject anything larger.
            if not -7.25 <= elevation_deg <= 52.25:
                raise ValueError(f"elevation outside MID-360 FOV tolerance at row {expected_index}")
            azimuth.append(azimuth_deg)
            elevation.append(elevation_deg)

    if len(azimuth) != EXPECTED_POINTS:
        raise ValueError(f"expected {EXPECTED_POINTS} trajectory points, found {len(azimuth)}")
    return Trajectory(tuple(azimuth), tuple(elevation))


def encode_trajectory(trajectory: Trajectory) -> bytes:
    """Quantize directions to Livox's 0.01-degree wire resolution."""

    values = array.array("H")
    for azimuth_deg, elevation_deg in zip(trajectory.azimuth_deg, trajectory.elevation_deg):
        values.append(round(azimuth_deg * 100.0) % 36_000)
        values.append(round((elevation_deg + 10.0) * 100.0))
    if sys.byteorder != "little":
        values.byteswap()
    return zlib.compress(values.tobytes(), level=9)


def _find_lidar(stage):
    candidates = [
        prim
        for prim in stage.TraverseAll()
        if prim.GetTypeName() == "OmniLidar" and str(prim.GetPath()).endswith(LIDAR_SUFFIX)
    ]
    if len(candidates) != 1:
        raise RuntimeError(f"expected one bundled MID-360 OmniLidar, found {len(candidates)}")
    return candidates[0]


def apply_profile(asset_path: Path, trajectory: Trajectory) -> dict[str, object]:
    """Replace the rotary approximation with a runtime-driven native profile."""

    from pxr import Sdf, Usd, Vt

    stage = Usd.Stage.Open(str(asset_path))
    if stage is None:
        raise RuntimeError(f"failed to open {asset_path}")
    lidar = _find_lidar(stage)

    for property_name in tuple(lidar.GetPropertyNames()):
        if property_name.startswith("omni:sensor:Core:emitterState:"):
            lidar.RemoveProperty(property_name)

    # Hydra in Isaac Sim 5.1 caps serialized attributes on one sensor at 5 MiB.
    # Keep one fixed-size RTX state on the OmniLidar and put the complete
    # compressed trajectory on a sibling Scope. The ROS/runtime driver swaps
    # values (never array lengths) at 10 Hz, which the schema supports.
    api_schemas = [
        "OmniSensorGenericLidarCoreAPI",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s001",
    ]
    lidar.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(api_schemas))

    def set_core(name: str, value) -> None:
        attribute = lidar.GetAttribute(f"omni:sensor:Core:{name}")
        if not attribute.IsValid():
            raise RuntimeError(f"missing core attribute {name!r} in {asset_path}")
        attribute.Set(value)

    set_core("scanType", "SOLID_STATE")
    set_core("scanRateBaseHz", SCAN_RATE_HZ)
    set_core("reportRateBaseHz", SCAN_RATE_HZ)
    set_core("numberOfEmitters", POINTS_PER_STATE)
    # The Core solid-state model represents every independently timed ray as
    # one detector channel (matching NVIDIA's bundled solid-state profiles).
    # Livox's CSV contains directions and ordering, but no physical laser ID.
    set_core("numberOfChannels", CHANNEL_COUNT)
    set_core("stateResolutionStep", 1)
    set_core("numLines", 1)
    set_core("numRaysPerLine", Vt.UIntArray([POINTS_PER_STATE]))
    set_core("startAzimuthOffsetDeg", 0.0)
    set_core("validStartAzimuthDeg", 0.0)
    set_core("validEndAzimuthDeg", 360.0)

    lidar.GetAttribute("omni:sensor:marketName").Set("Livox MID-360")
    lidar.GetAttribute("omni:sensor:modelName").Set("MID-360 non-repetitive pattern replay")
    lidar.GetAttribute("omni:sensor:modelVendor").Set("Livox")
    lidar.GetAttribute("omni:sensor:modelVersion").Set("IsaacSim-5.1-official-trace-4s")
    lidar.GetAttribute("omni:sensor:tickRate").Set(float(SCAN_RATE_HZ))

    fire_times = Vt.UIntArray([index * POINT_INTERVAL_NS for index in range(POINTS_PER_STATE)])
    # The USD-native Core model validates channel IDs as 1-based.
    channel_ids = Vt.UIntArray([index + 1 for index in range(POINTS_PER_STATE)])
    banks = Vt.UIntArray([0] * POINTS_PER_STATE)
    azimuth, elevation = next(iter(trajectory.states()))
    signed_azimuth = [value if value < 180.0 else value - 360.0 for value in azimuth]
    prefix = "omni:sensor:Core:emitterState:s001:"
    values = (
        ("azimuthDeg", Sdf.ValueTypeNames.FloatArray, Vt.FloatArray(signed_azimuth)),
        ("elevationDeg", Sdf.ValueTypeNames.FloatArray, Vt.FloatArray(elevation)),
        ("fireTimeNs", Sdf.ValueTypeNames.UIntArray, fire_times),
        ("channelId", Sdf.ValueTypeNames.UIntArray, channel_ids),
        ("bank", Sdf.ValueTypeNames.UIntArray, banks),
    )
    for field, value_type, value in values:
        lidar.CreateAttribute(prefix + field, value_type, custom=False).Set(value)

    pattern_path = lidar.GetParent().GetPath().AppendChild(PATTERN_DATA_NAME)
    stage.RemovePrim(pattern_path)
    pattern = stage.DefinePrim(pattern_path, "Scope")

    def set_pattern(name: str, value_type, value) -> None:
        pattern.CreateAttribute(f"lidarHiking:{name}", value_type, custom=True).Set(value)

    compressed_directions = encode_trajectory(trajectory)
    set_pattern("encoding", Sdf.ValueTypeNames.String, PATTERN_ENCODING)
    set_pattern("compressedDirections", Sdf.ValueTypeNames.UCharArray, Vt.UCharArray(compressed_directions))
    set_pattern("pointRateHz", Sdf.ValueTypeNames.UInt, POINT_RATE_HZ)
    set_pattern("scanRateHz", Sdf.ValueTypeNames.UInt, SCAN_RATE_HZ)
    set_pattern("pointsPerState", Sdf.ValueTypeNames.UInt, POINTS_PER_STATE)
    set_pattern("trajectoryStates", Sdf.ValueTypeNames.UInt, STATE_COUNT)
    set_pattern("trajectorySha256", Sdf.ValueTypeNames.String, EXPECTED_TRAJECTORY_SHA256)

    # Sdf crate Save() appends changed data and grows on every regeneration.
    # Export to a fresh crate and atomically replace the asset so this command
    # stays size-stable and deterministic.
    temporary_asset = asset_path.with_name(asset_path.stem + ".profile-tmp.usd")
    if not stage.GetRootLayer().Export(str(temporary_asset)):
        raise RuntimeError(f"failed to export compact profile asset {temporary_asset}")
    os.replace(temporary_asset, asset_path)
    return {
        "asset": str(asset_path.relative_to(ROOT)),
        "lidar_prim": str(lidar.GetPath()),
        "scan_type": "SOLID_STATE",
        "scan_rate_hz": SCAN_RATE_HZ,
        "rtx_emitter_states": EMBEDDED_EMITTER_STATE_COUNT,
        "trajectory_states": STATE_COUNT,
        "emitters_per_state": POINTS_PER_STATE,
        "points_per_second": POINT_RATE_HZ,
        "trajectory_duration_s": EXPECTED_POINTS / POINT_RATE_HZ,
        "compressed_pattern_bytes": len(compressed_directions),
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, default=TRAJECTORY)
    parser.add_argument("assets", type=Path, nargs="*", default=list(ASSETS))
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    trajectory = load_trajectory(args.trajectory.resolve())
    simulation_app = None
    try:
        # Isaac Sim 5.1 exposes pxr after Kit has initialized, even when this
        # script is launched through python.sh.
        try:
            import pxr  # noqa: F401
        except ModuleNotFoundError:
            from isaacsim import SimulationApp

            simulation_app = SimulationApp({"headless": True})
        results = [apply_profile(asset.resolve(), trajectory) for asset in args.assets]
        print(json.dumps({"updated": results}, indent=2, sort_keys=True))
    finally:
        if simulation_app is not None:
            simulation_app.close()


if __name__ == "__main__":
    main()
