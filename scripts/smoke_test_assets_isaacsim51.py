#!/usr/bin/env python3
"""Open and validate all published assets inside Isaac Sim 5.1."""

from __future__ import annotations

import json
import traceback
from pathlib import Path

from isaacsim import SimulationApp


APP = SimulationApp({"headless": True})

import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
CATALOG = json.loads((ROOT / "assets/catalog.json").read_text(encoding="utf-8"))
LIDAR_SUFFIX = "/torso_link/mid360_link/mid360_native_approx"


def _one_lidar(stage: Usd.Stage):
    lidars = [
        prim
        for prim in stage.TraverseAll()
        if prim.GetTypeName() == "OmniLidar" and str(prim.GetPath()).endswith(LIDAR_SUFFIX)
    ]
    if len(lidars) != 1:
        raise RuntimeError(f"expected one MID-360 OmniLidar, found {len(lidars)}")
    return lidars[0]


def _same_transform(left, right) -> bool:
    return max(
        abs(float(left[row][column] - right[row][column]))
        for row in range(4)
        for column in range(4)
    ) <= 1.0e-6


def validate_asset(asset_id: str, entry: dict[str, object]) -> None:
    path = ROOT / str(entry["path"])
    stage = Usd.Stage.Open(str(path))
    if stage is None or not stage.GetDefaultPrim().IsValid():
        raise RuntimeError(f"could not open {path}")
    lidar = _one_lidar(stage)
    core = "omni:sensor:Core:"
    scan_type = str(lidar.GetAttribute(core + "scanType").Get())
    emitters = int(lidar.GetAttribute(core + "numberOfEmitters").Get())
    scan_rate = int(lidar.GetAttribute(core + "scanRateBaseHz").Get())
    pattern = lidar.GetParent().GetChild("mid360_nonrepetitive_pattern")
    expected_profile = str(entry["profile"])
    if expected_profile == "petal_scan":
        if (scan_type, emitters, scan_rate) != ("SOLID_STATE", 20_000, 10):
            raise RuntimeError(f"bad Petal profile in {asset_id}")
        if not pattern.IsValid():
            raise RuntimeError(f"missing Petal trajectory in {asset_id}")
        states = int(pattern.GetAttribute("lidarHiking:trajectoryStates").Get())
        points_per_state = int(pattern.GetAttribute("lidarHiking:pointsPerState").Get())
        if (states, points_per_state) != (40, 20_000):
            raise RuntimeError(f"bad Petal state contract in {asset_id}")
    else:
        report_rate = int(lidar.GetAttribute(core + "reportRateBaseHz").Get())
        if (scan_type, emitters, scan_rate, report_rate) != ("ROTARY", 40, 10, 5_000):
            raise RuntimeError(f"bad Rotary profile in {asset_id}")
        if pattern.IsValid():
            raise RuntimeError(f"Rotary asset unexpectedly embeds Petal trajectory: {asset_id}")

    joint_count = sum(1 for prim in stage.Traverse() if prim.IsA(UsdPhysics.RevoluteJoint))
    if entry["form"] == "robot":
        if joint_count != 29:
            raise RuntimeError(f"{asset_id} has {joint_count} revolute joints")
        mount = lidar.GetParent()
        robot_root_path = str(lidar.GetPath())[: -len(LIDAR_SUFFIX)]
        imu = stage.GetPrimAtPath(robot_root_path + "/torso_link/mid360_imu")
        if not imu.IsValid():
            imu_candidates = [
                str(prim.GetPath())
                for prim in stage.TraverseAll()
                if prim.GetTypeName() == "IsaacImuSensor" or "mid360_imu" in str(prim.GetPath())
            ]
            raise RuntimeError(f"missing matching IMU in {asset_id}; candidates={imu_candidates}")
        mount_matrix = UsdGeom.Xformable(mount).GetLocalTransformation()
        imu_matrix = UsdGeom.Xformable(imu).GetLocalTransformation()
        if not _same_transform(mount_matrix, imu_matrix):
            raise RuntimeError(f"LiDAR/IMU transform mismatch in {asset_id}")
        sensor_up = mount_matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)).GetNormalized()
        if sensor_up[2] > -0.999:
            raise RuntimeError(f"robot MID-360 is not mounted with 180-degree roll: {asset_id}")
    else:
        if joint_count != 0:
            raise RuntimeError(f"standalone asset contains robot joints: {asset_id}")
        torso = stage.GetPrimAtPath("/MID360/torso_link")
        rigid_body = UsdPhysics.RigidBodyAPI(torso)
        if not rigid_body or not rigid_body.GetRigidBodyEnabledAttr().Get():
            raise RuntimeError(f"standalone IMU parent is not a rigid body: {asset_id}")
        if not rigid_body.GetKinematicEnabledAttr().Get():
            raise RuntimeError(f"standalone MID-360 rigid body is not fixed: {asset_id}")

    print(
        f"ASSET_OK={asset_id} PROFILE={expected_profile} FORM={entry['form']} "
        f"SCAN_TYPE={scan_type} EMITTERS={emitters} JOINTS={joint_count}",
        flush=True,
    )


def render_smoke(asset_id: str, entry: dict[str, object]) -> None:
    context = omni.usd.get_context()
    context.open_stage(str(ROOT / str(entry["path"])))
    for _ in range(8):
        APP.update()
    stage = context.get_stage()
    lidar = _one_lidar(stage)
    render_product = rep.create.render_product(
        str(lidar.GetPath()),
        resolution=(32, 32),
        name=f"{asset_id.replace('-', '_')}_SmokeRenderProduct",
        render_vars=["GenericModelOutput", "RtxSensorMetadata"],
    )
    for _ in range(12):
        APP.update()
    render_product.destroy()
    for _ in range(3):
        APP.update()
    print(f"RENDER_OK={asset_id}", flush=True)


def main() -> None:
    try:
        for asset_id, entry in CATALOG["assets"].items():
            validate_asset(asset_id, entry)
        for asset_id in ("mid360-petal-standalone", "mid360-rotary-standalone"):
            render_smoke(asset_id, CATALOG["assets"][asset_id])
        print(f"ISAACSIM_ASSET_COUNT={len(CATALOG['assets'])}", flush=True)
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        APP.close()


main()
