#!/usr/bin/env python3
"""Run one fixed G1-5010 object-field scene and publish MID-360 PointCloud2."""

from __future__ import annotations

import argparse
import os
import time
import traceback
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENES = {
    "petal": ROOT
    / "tests/scenes/object_field/MID360_G1_5010_Petal_Object_Field.usda",
    "rotary": ROOT
    / "tests/scenes/object_field/MID360_G1_5010_Rotary_Object_Field.usda",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=tuple(SCENES), default="petal")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="seconds to keep the GUI running; 0 means until the window closes",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scene_path = SCENES[args.profile]
    if not scene_path.is_file():
        raise FileNotFoundError(
            f"missing {scene_path}; run scripts/build_object_field_validation_scenes.py first"
        )

    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": False,
            "width": 1440,
            "height": 900,
            "sync_loads": True,
        }
    )

    import carb.settings
    import omni.kit.app
    import omni.timeline
    import omni.usd

    publisher_namespace: dict[str, object] = {}
    try:
        print(f"VALIDATION_BOOT profile={args.profile} scene={scene_path}", flush=True)
        settings = carb.settings.get_settings()
        settings.set("/app/sensors/nv/lidar/outputBufferOnGPU", True)
        settings.set("/app/window/title", f"MID360 5010 {args.profile.title()} Object Field")

        extension_manager = omni.kit.app.get_app().get_extension_manager()
        extension_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
        for _ in range(60):
            app.update()
        print("VALIDATION_ROS_BRIDGE_READY", flush=True)

        context = omni.usd.get_context()
        context.open_stage(str(scene_path))
        # Isaac Sim 5.1 removed UsdContext.is_stage_loading().  sync_loads plus
        # a bounded update loop keeps this launcher compatible with 5.1.
        for _ in range(180):
            app.update()
        stage = context.get_stage()
        if stage is None or not stage.GetPrimAtPath("/World/G1").IsValid():
            raise RuntimeError(f"could not open {scene_path}")
        print("VALIDATION_STAGE_READY robot=/World/G1", flush=True)

        viewport = None
        try:
            from omni.kit.viewport.utility import get_active_viewport

            viewport = get_active_viewport()
            if viewport is not None:
                viewport.set_active_camera("/World/ValidationCamera")
        except Exception as error:
            print(f"[ObjectFieldValidation] viewport camera warning: {error}", flush=True)

        global MID360_ROBOT_ROOT_PATH
        MID360_ROBOT_ROOT_PATH = "/World/G1"
        publisher_path = ROOT / "scripts/publish_mid360_ros2_isaacsim51.py"
        publisher_namespace = {
            "__file__": str(publisher_path),
            "__name__": "mid360_object_field_publisher",
        }
        code = publisher_path.read_text(encoding="utf-8")
        print("VALIDATION_PUBLISHER_START", flush=True)
        exec(compile(code, str(publisher_path), "exec"), publisher_namespace)
        print("VALIDATION_PUBLISHER_READY", flush=True)

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        print(
            f"VALIDATION_READY profile={args.profile} scene={scene_path} "
            "topic=/mid360/points frame=mid360_link joints_locked=29",
            flush=True,
        )

        start = time.monotonic()
        while app.is_running():
            app.update()
            if args.duration > 0.0 and time.monotonic() - start >= args.duration:
                break
        return 0
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        cleanup = publisher_namespace.get("cleanup_mid360_ros2")
        if callable(cleanup):
            try:
                cleanup()
            except Exception as error:
                print(f"[ObjectFieldValidation] cleanup warning: {error}", flush=True)
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
