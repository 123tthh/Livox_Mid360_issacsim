#!/usr/bin/env python3
"""Build a standalone MID-360 USD from the bundled sensor and a CAD USD."""

from __future__ import annotations

import argparse
import os
from pathlib import Path


def _bootstrap_pxr():
    try:
        from pxr import Gf, Kind, Sdf, Usd, UsdGeom

        return None, Gf, Kind, Sdf, Usd, UsdGeom
    except ModuleNotFoundError:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True})
        from pxr import Gf, Kind, Sdf, Usd, UsdGeom

        return app, Gf, Kind, Sdf, Usd, UsdGeom


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("cad_usd", type=Path)
    parser.add_argument("output_usd", type=Path)
    parser.add_argument(
        "--sensor-source",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "assets/g1_29dof_rev_1_0/G1_29dof_mid360.usd",
    )
    args = parser.parse_args()

    app, Gf, Kind, Sdf, Usd, UsdGeom = _bootstrap_pxr()
    try:
        source = Usd.Stage.Open(str(args.sensor_source.resolve()))
        if source is None:
            raise RuntimeError(f"Could not open sensor source {args.sensor_source}")
        args.output_usd.parent.mkdir(parents=True, exist_ok=True)
        stage = Usd.Stage.CreateNew(str(args.output_usd.resolve()))
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        root = UsdGeom.Xform.Define(stage, "/MID360").GetPrim()
        stage.SetDefaultPrim(root)
        root.SetAssetInfoByKey("name", "Livox MID-360 non-repetitive RTX LiDAR")
        root.SetAssetInfoByKey("version", "Isaac Sim 5.1")
        Usd.ModelAPI(root).SetKind(Kind.Tokens.component)
        UsdGeom.Xform.Define(stage, "/MID360/torso_link")
        UsdGeom.Xform.Define(stage, "/MID360/torso_link/mid360_link")

        source_root = source.GetRootLayer()
        target_root = stage.GetRootLayer()
        copies = (
            (
                "/g1_29dof_rev_1_0/torso_link/mid360_link/mid360_native_approx",
                "/MID360/torso_link/mid360_link/mid360_native_approx",
            ),
            (
                "/g1_29dof_rev_1_0/torso_link/mid360_link/mid360_nonrepetitive_pattern",
                "/MID360/torso_link/mid360_link/mid360_nonrepetitive_pattern",
            ),
            (
                "/g1_29dof_rev_1_0/torso_link/mid360_imu",
                "/MID360/torso_link/mid360_imu",
            ),
        )
        for source_path, target_path in copies:
            if source.GetPrimAtPath(source_path).IsValid():
                if not Sdf.CopySpec(source_root, source_path, target_root, target_path):
                    raise RuntimeError(f"Could not copy {source_path}")

        cad = UsdGeom.Xform.Define(stage, "/MID360/torso_link/mid360_link/CAD")
        relative_cad = os.path.relpath(args.cad_usd.resolve(), args.output_usd.resolve().parent)
        cad.GetPrim().GetReferences().AddReference(relative_cad)
        # HOOPS preserves STEP coordinates in millimetres.  The wrapper is in metres.
        cad.AddScaleOp().Set(Gf.Vec3f(0.001, 0.001, 0.001))

        stage.GetRootLayer().Save()
        print(f"STANDALONE_USD={args.output_usd.resolve()}", flush=True)
        return 0
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    raise SystemExit(main())
