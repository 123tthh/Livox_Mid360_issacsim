#!/usr/bin/env python3
"""Build the self-contained object field and fixed G1-5010 validation stages."""

from __future__ import annotations

import os
import traceback
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "tests/scenes/object_field"
ENVIRONMENT_PATH = OUTPUT_DIR / "MID360_Object_Field.usda"
ROBOT_ROOT_HEIGHT_M = 0.836273

PROFILES = {
    "petal": {
        "asset": ROOT
        / "assets/petal_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Petal_Scan.usd",
        "stage": OUTPUT_DIR / "MID360_G1_5010_Petal_Object_Field.usda",
        "label": "Petal Scan",
    },
    "rotary": {
        "asset": ROOT
        / "assets/rotary_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Rotary_Scan.usd",
        "stage": OUTPUT_DIR / "MID360_G1_5010_Rotary_Object_Field.usda",
        "label": "Rotary Scan",
    },
}

JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
    "waist_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)


def _bootstrap_pxr():
    try:
        from pxr import Gf, Kind, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

        return None, Gf, Kind, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics
    except ModuleNotFoundError:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True})
        from pxr import Gf, Kind, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

        return app, Gf, Kind, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics


def _set_color(geom, Gf, rgb: tuple[float, float, float]) -> None:
    geom.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])


def _translate(geom, Gf, xyz: tuple[float, float, float]) -> None:
    translate_op = geom.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(*xyz))
    ordered_ops = geom.GetOrderedXformOps()
    if ordered_ops and ordered_ops[0] != translate_op:
        geom.SetXformOpOrder([translate_op] + [op for op in ordered_ops if op != translate_op])


def _static_collision(prim, UsdPhysics) -> None:
    UsdPhysics.CollisionAPI.Apply(prim)


def build_environment(Gf, Kind, Sdf, Usd, UsdGeom, UsdPhysics) -> None:
    stage = Usd.Stage.CreateNew(str(ENVIRONMENT_PATH))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/ObjectField").GetPrim()
    stage.SetDefaultPrim(root)
    Usd.ModelAPI(root).SetKind(Kind.Tokens.component)
    root.CreateAttribute("mid360Validation:radiusM", Sdf.ValueTypeNames.Double).Set(5.0)
    root.CreateAttribute("mid360Validation:objectCount", Sdf.ValueTypeNames.Int).Set(10)
    root.CreateAttribute("mid360Validation:allObjectsTouchGround", Sdf.ValueTypeNames.Bool).Set(True)
    root.CreateAttribute("mid360Validation:projectionWallDistanceM", Sdf.ValueTypeNames.Double).Set(4.75)

    ground = UsdGeom.Cube.Define(stage, "/ObjectField/Ground")
    ground.CreateSizeAttr(1.0)
    ground.AddScaleOp().Set(Gf.Vec3f(12.0, 12.0, 0.1))
    _translate(ground, Gf, (0.0, 0.0, -0.05))
    _set_color(ground, Gf, (0.18, 0.20, 0.22))
    _static_collision(ground.GetPrim(), UsdPhysics)

    projection_wall = UsdGeom.Cube.Define(stage, "/ObjectField/Objects/ProjectionWall")
    projection_wall.CreateSizeAttr(1.0)
    projection_wall.AddScaleOp().Set(Gf.Vec3f(5.5, 0.10, 3.2))
    _translate(projection_wall, Gf, (0.0, 4.75, 1.6))
    _set_color(projection_wall, Gf, (0.62, 0.66, 0.70))
    projection_wall.GetPrim().CreateAttribute(
        "mid360Validation:angularProjectionTarget", Sdf.ValueTypeNames.Bool
    ).Set(True)
    _static_collision(projection_wall.GetPrim(), UsdPhysics)

    box = UsdGeom.Cube.Define(stage, "/ObjectField/Objects/Box")
    box.CreateSizeAttr(1.0)
    box.AddScaleOp().Set(Gf.Vec3f(1.1, 0.8, 1.0))
    _translate(box, Gf, (2.2, 0.4, 0.5))
    _set_color(box, Gf, (0.86, 0.18, 0.15))
    _static_collision(box.GetPrim(), UsdPhysics)

    sphere = UsdGeom.Sphere.Define(stage, "/ObjectField/Objects/Sphere")
    sphere.CreateRadiusAttr(0.55)
    _translate(sphere, Gf, (-2.0, 1.6, 0.55))
    _set_color(sphere, Gf, (0.16, 0.48, 0.92))
    _static_collision(sphere.GetPrim(), UsdPhysics)

    cylinder = UsdGeom.Cylinder.Define(stage, "/ObjectField/Objects/Cylinder")
    cylinder.CreateAxisAttr(UsdGeom.Tokens.z)
    cylinder.CreateRadiusAttr(0.45)
    cylinder.CreateHeightAttr(1.5)
    _translate(cylinder, Gf, (0.2, 3.0, 0.75))
    _set_color(cylinder, Gf, (0.15, 0.78, 0.33))
    _static_collision(cylinder.GetPrim(), UsdPhysics)

    cone = UsdGeom.Cone.Define(stage, "/ObjectField/Objects/Cone")
    cone.CreateAxisAttr(UsdGeom.Tokens.z)
    cone.CreateRadiusAttr(0.7)
    cone.CreateHeightAttr(1.4)
    _translate(cone, Gf, (-3.0, -1.2, 0.7))
    _set_color(cone, Gf, (0.96, 0.55, 0.10))
    _static_collision(cone.GetPrim(), UsdPhysics)

    capsule = UsdGeom.Capsule.Define(stage, "/ObjectField/Objects/Capsule")
    capsule.CreateAxisAttr(UsdGeom.Tokens.z)
    capsule.CreateRadiusAttr(0.35)
    capsule.CreateHeightAttr(0.9)
    _translate(capsule, Gf, (1.4, -3.0, 0.8))
    _set_color(capsule, Gf, (0.62, 0.27, 0.88))
    _static_collision(capsule.GetPrim(), UsdPhysics)

    wall = UsdGeom.Cube.Define(stage, "/ObjectField/Objects/ThinWall")
    wall.CreateSizeAttr(1.0)
    wall.AddScaleOp().Set(Gf.Vec3f(1.5, 0.22, 2.0))
    _translate(wall, Gf, (3.5, 2.2, 1.0))
    _set_color(wall, Gf, (0.10, 0.74, 0.78))
    _static_collision(wall.GetPrim(), UsdPhysics)

    pyramid = UsdGeom.Mesh.Define(stage, "/ObjectField/Objects/Pyramid")
    pyramid.CreatePointsAttr(
        [
            Gf.Vec3f(-0.65, -0.65, 0.0),
            Gf.Vec3f(0.65, -0.65, 0.0),
            Gf.Vec3f(0.65, 0.65, 0.0),
            Gf.Vec3f(-0.65, 0.65, 0.0),
            Gf.Vec3f(0.0, 0.0, 1.8),
        ]
    )
    pyramid.CreateFaceVertexCountsAttr([4, 3, 3, 3, 3])
    pyramid.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4])
    pyramid.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    pyramid.CreateExtentAttr([Gf.Vec3f(-0.65, -0.65, 0.0), Gf.Vec3f(0.65, 0.65, 1.8)])
    _translate(pyramid, Gf, (-3.4, 2.5, 0.0))
    _set_color(pyramid, Gf, (0.93, 0.25, 0.62))
    _static_collision(pyramid.GetPrim(), UsdPhysics)
    UsdPhysics.MeshCollisionAPI.Apply(pyramid.GetPrim()).CreateApproximationAttr("none")

    wedge = UsdGeom.Mesh.Define(stage, "/ObjectField/Objects/Wedge")
    wedge.CreatePointsAttr(
        [
            Gf.Vec3f(-0.8, -0.6, 0.0),
            Gf.Vec3f(0.8, -0.6, 0.0),
            Gf.Vec3f(0.8, 0.6, 0.0),
            Gf.Vec3f(-0.8, 0.6, 0.0),
            Gf.Vec3f(0.8, -0.6, 0.9),
            Gf.Vec3f(0.8, 0.6, 0.9),
        ]
    )
    wedge.CreateFaceVertexCountsAttr([4, 4, 3, 3, 4])
    wedge.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 1, 2, 5, 4, 0, 1, 4, 3, 5, 2, 0, 4, 5, 3])
    wedge.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    wedge.CreateExtentAttr([Gf.Vec3f(-0.8, -0.6, 0.0), Gf.Vec3f(0.8, 0.6, 0.9)])
    _translate(wedge, Gf, (-1.0, -4.0, 0.0))
    _set_color(wedge, Gf, (0.85, 0.82, 0.13))
    _static_collision(wedge.GetPrim(), UsdPhysics)
    UsdPhysics.MeshCollisionAPI.Apply(wedge.GetPrim()).CreateApproximationAttr("none")

    arch = UsdGeom.Xform.Define(stage, "/ObjectField/Objects/Arch")
    _translate(arch, Gf, (3.6, -2.5, 0.0))
    for name, xyz, scale in (
        ("LeftPost", (-0.55, 0.0, 0.8), (0.28, 0.45, 1.6)),
        ("RightPost", (0.55, 0.0, 0.8), (0.28, 0.45, 1.6)),
        ("Lintel", (0.0, 0.0, 1.75), (1.38, 0.45, 0.3)),
    ):
        part = UsdGeom.Cube.Define(stage, f"/ObjectField/Objects/Arch/{name}")
        part.CreateSizeAttr(1.0)
        part.AddScaleOp().Set(Gf.Vec3f(*scale))
        _translate(part, Gf, xyz)
        _set_color(part, Gf, (0.48, 0.30, 0.16))
        _static_collision(part.GetPrim(), UsdPhysics)

    stage.GetRootLayer().Save()


def build_stage(profile: str, Gf, Kind, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics) -> None:
    spec = PROFILES[profile]
    output = spec["stage"]
    print(f"BUILDING={profile} STAGE={output}", flush=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(root)
    Usd.ModelAPI(root).SetKind(Kind.Tokens.assembly)
    root.CreateAttribute("mid360Validation:profile", Sdf.ValueTypeNames.String).Set(profile)
    root.CreateAttribute("mid360Validation:robot", Sdf.ValueTypeNames.String).Set("G1 5010 Mode13")
    root.CreateAttribute("mid360Validation:lockedJointCount", Sdf.ValueTypeNames.Int).Set(29)
    root.CreateAttribute("mid360Validation:usesExternalControl", Sdf.ValueTypeNames.Bool).Set(False)

    environment = UsdGeom.Xform.Define(stage, "/World/ObjectField")
    environment.GetPrim().GetReferences().AddReference(
        os.path.relpath(ENVIRONMENT_PATH, output.parent)
    )

    robot = UsdGeom.Xform.Define(stage, "/World/G1")
    robot.GetPrim().GetReferences().AddReference(os.path.relpath(spec["asset"], output.parent))
    robot.GetPrim().GetAttribute("xformOp:translate").Set(
        Gf.Vec3d(0.0, 0.0, ROBOT_ROOT_HEIGHT_M)
    )

    for joint_name in JOINT_NAMES:
        joint = stage.OverridePrim(f"/World/G1/joints/{joint_name}")
        joint.CreateAttribute("physics:lowerLimit", Sdf.ValueTypeNames.Float, custom=False).Set(0.0)
        joint.CreateAttribute("physics:upperLimit", Sdf.ValueTypeNames.Float, custom=False).Set(0.0)
        joint.CreateAttribute(
            "drive:angular:physics:targetPosition", Sdf.ValueTypeNames.Float, custom=False
        ).Set(0.0)

    anchor = UsdPhysics.FixedJoint.Define(stage, "/World/G1WorldAnchor")
    anchor.CreateBody1Rel().SetTargets([Sdf.Path("/World/G1/torso_link")])
    anchor.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, ROBOT_ROOT_HEIGHT_M))
    anchor.CreateLocalRot0Attr(Gf.Quatf(1.0))
    anchor.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    anchor.CreateLocalRot1Attr(Gf.Quatf(1.0))

    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics.CreateGravityMagnitudeAttr(9.81)

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(650.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(3000.0)
    sun.CreateAngleAttr(0.53)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(315.0, 0.0, 35.0))

    camera = UsdGeom.Camera.Define(stage, "/World/ValidationCamera")
    camera.CreateFocalLengthAttr(24.0)
    camera_matrix = Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(-8.5, -8.5, 5.2),
        Gf.Vec3d(0.0, 0.0, 0.8),
        Gf.Vec3d(0.0, 0.0, 1.0),
    ).GetInverse()
    camera.MakeMatrixXform().Set(camera_matrix)

    stage.GetRootLayer().Save()
    print(f"BUILT={profile} STAGE={output}", flush=True)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    app, Gf, Kind, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics = _bootstrap_pxr()
    try:
        build_environment(Gf, Kind, Sdf, Usd, UsdGeom, UsdPhysics)
        print(f"BUILT=environment STAGE={ENVIRONMENT_PATH}", flush=True)
        for profile in PROFILES:
            build_stage(profile, Gf, Kind, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics)
        return 0
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    raise SystemExit(main())
