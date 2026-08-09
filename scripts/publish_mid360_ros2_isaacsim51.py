"""Publish a G1-mounted MID360 Petal or Rotary scan as ROS 2 PointCloud2.

Run this whole file from Window > Script Editor after opening either bundled
G1 USD in Isaac Sim 5.1 with the ROS 2 Bridge enabled. It builds this graph:

    On Playback Tick -> ROS2 RTX Lidar Helper -> /mid360/points
                     -> ROS2 Transform Tree   -> /tf

The graph lives in the USD session layer and is not saved into the robot asset.
Before it creates the graph, this script also verifies the independent
``ROS_RobotState`` and ``ROS_Mid360Imu`` graphs use their own simulation-time
nodes. The override is session-only, but it prevents a running scene from
mixing Unix time with ``/clock``. Unlike the upstream sensor-only helper, this
scene variant never disables gravity.  It does not load a policy, controller,
localization or mapping stack.
This publisher must be the sole owner of its RTX render product; sharing RTX
LiDAR resources can raise ``cudaErrorInvalidValue`` in Isaac Sim 5.1.

Isaac Sim 5.1.0-rc.19 is unstable with the ROS 2 helper's accumulated
``fullScan=True`` path for this custom OmniLidar and can segfault shortly after
playback starts. This script publishes non-accumulated per-render-frame slices.
RViz can accumulate the slices for a short display interval; no frame skipping
or point subsampling is used by this publisher.

Run ``cleanup_mid360_ros2()`` in Script Editor to stop publishing and remove the
temporary graph/render product.
"""

from __future__ import annotations

import array
import sys
import zlib

import carb
import carb.settings
import omni.graph.core as og
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
import usdrt
from pxr import Gf, Sdf, Usd, UsdGeom, Vt

import __main__

LIDAR_SUFFIX = "/torso_link/mid360_link/mid360_native_approx"
MOUNT_SUFFIX = "/torso_link/mid360_link"
IMU_SUFFIX = "/torso_link/mid360_imu"
PATTERN_SUFFIX = "/torso_link/mid360_link/mid360_nonrepetitive_pattern"
ROBOT_ROOT_PATH = "/g1_29dof_mode_13_5010_mid360"
MOUNT_PATH = ROBOT_ROOT_PATH + MOUNT_SUFFIX
LIDAR_PATH = ROBOT_ROOT_PATH + LIDAR_SUFFIX
IMU_PATH = ROBOT_ROOT_PATH + IMU_SUFFIX
PATTERN_PATH = ROBOT_ROOT_PATH + PATTERN_SUFFIX
GRAPH_PATH = ROBOT_ROOT_PATH + "/ROS_Mid360PointCloud"
TOPIC_NAME = "mid360/points"
FRAME_ID = "mid360_link"
ROBOT_FIXED_FRAME_ID = "g1_29dof_mode_13_5010_mid360"
ROS_STATE_GRAPH_PATH = "/World/Graphs/ROS_RobotState"
ROS_MID360_IMU_GRAPH_PATH = "/World/Graphs/ROS_Mid360Imu"
SIMULATION_TIME_CONTRACTS = (
    (
        ROS_STATE_GRAPH_PATH,
        (
            "PublishJointState",
            "PublishPolicyImu",
            "PublishGroundTruthOdometry",
            "PublishClock",
        ),
    ),
    (ROS_MID360_IMU_GRAPH_PATH, ("PublishMid360Imu",)),
)

# Keep the official helper default.  fullScan=True selects the accumulated scan
# buffer path that crashes Isaac Sim 5.1.0-rc.19 with this custom OmniLidar.
# With False, one current RTX slice is published per render update; the input
# a visualization or consumer may combine slices into a stable 10 Hz scan.
PUBLISH_FULL_SCAN = False
PUBLISH_FRAME_SKIP_COUNT = 0
USE_SYSTEM_TIME = False
QUEUE_SIZE = 10
AUTO_START_TIMELINE = False
PATTERN_ENCODING = "zlib+u16le:azimuth_centideg,elevation_centideg_plus_1000"
PATTERN_ATTRIBUTE_NAMES = (
    "omni:sensor:Core:emitterState:s001:azimuthDeg",
    "omni:sensor:Core:emitterState:s001:elevationDeg",
)


def _resolve_robot_instance(stage: Usd.Stage) -> None:
    """Resolve one reusable G1+Mid360 instance in the current scene.

    When a scene contains multiple instances, set ``MID360_ROBOT_ROOT_PATH``
    in Script Editor before running this file, for example::

        MID360_ROBOT_ROOT_PATH = "/World/G1_A"
    """

    hint = getattr(__main__, "MID360_ROBOT_ROOT_PATH", None)
    if hint:
        root_path = str(Sdf.Path(str(hint)))
        candidate = stage.GetPrimAtPath(root_path + LIDAR_SUFFIX)
        if not candidate.IsValid() or candidate.GetTypeName() != "OmniLidar":
            raise RuntimeError(
                f"MID360_ROBOT_ROOT_PATH={root_path!r} does not contain an OmniLidar at {root_path + LIDAR_SUFFIX}"
            )
        candidates = [candidate]
    else:
        candidates = [
            prim
            for prim in stage.TraverseAll()
            if prim.GetTypeName() == "OmniLidar" and str(prim.GetPath()).endswith(LIDAR_SUFFIX)
        ]

    if not candidates:
        raise RuntimeError(
            "No G1 MID-360 instance found. Add/reference one of the standardized "
            "Petal Scan or Rotary Scan robot assets first."
        )
    if len(candidates) > 1:
        roots = [str(prim.GetPath())[: -len(LIDAR_SUFFIX)] for prim in candidates]
        raise RuntimeError(f"Multiple G1 Mid360 instances found. Set MID360_ROBOT_ROOT_PATH to one of: {roots}")

    lidar_path = str(candidates[0].GetPath())
    robot_root_path = lidar_path[: -len(LIDAR_SUFFIX)]
    global ROBOT_ROOT_PATH, MOUNT_PATH, LIDAR_PATH, IMU_PATH, PATTERN_PATH, GRAPH_PATH
    global ROBOT_FIXED_FRAME_ID
    ROBOT_ROOT_PATH = robot_root_path
    MOUNT_PATH = robot_root_path + MOUNT_SUFFIX
    LIDAR_PATH = lidar_path
    IMU_PATH = robot_root_path + IMU_SUFFIX
    PATTERN_PATH = robot_root_path + PATTERN_SUFFIX
    GRAPH_PATH = robot_root_path + "/ROS_Mid360PointCloud"
    ROBOT_FIXED_FRAME_ID = Sdf.Path(robot_root_path).name
    print(f"[Mid360ROS2] Reusable asset instance resolved: {ROBOT_ROOT_PATH}; runtime graph: {GRAPH_PATH}")


def _remove_spec(layer: Sdf.Layer, path: Sdf.Path) -> None:
    if layer.GetPrimAtPath(path) is None:
        return
    edits = Sdf.BatchNamespaceEdit()
    edits.Add(Sdf.NamespaceEdit.Remove(path))
    if not layer.Apply(edits):
        raise RuntimeError(f"Could not remove temporary graph {path}")


def _clear_selection() -> None:
    """Avoid Kit's invalid-null-prim manipulator error during graph deletion."""

    selection = omni.usd.get_context().get_selection()
    selection.clear_selected_prim_paths()
    selection.set_selected_prim_paths([], False)
    try:
        selection.clear_selected_prim_paths(omni.usd.Selection.SourceType.FABRIC)
    except (AttributeError, TypeError):
        pass


def _validate_lidar(stage: Usd.Stage) -> str:
    lidar = stage.GetPrimAtPath(LIDAR_PATH)
    if not lidar.IsValid() or lidar.GetTypeName() != "OmniLidar":
        raise RuntimeError(f"No OmniLidar at {LIDAR_PATH}. Run add_mid360_native_approx_isaacsim51.py first.")

    applied_schemas = set(lidar.GetAppliedSchemas())
    required_schemas = {
        "OmniSensorGenericLidarCoreAPI",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s001",
    }
    missing_schemas = required_schemas - applied_schemas
    if missing_schemas:
        raise RuntimeError(
            "OmniLidar is missing required Isaac Sim 5.1 schemas: "
            f"{sorted(missing_schemas)}. Rebuild it with "
            "add_mid360_native_approx_isaacsim51.py."
        )

    core = "omni:sensor:Core:"
    emitters = lidar.GetAttribute(core + "numberOfEmitters").Get()
    scan_type = str(lidar.GetAttribute(core + "scanType").Get())
    report_rate = lidar.GetAttribute(core + "reportRateBaseHz").Get()
    pattern = stage.GetPrimAtPath(PATTERN_PATH)
    if scan_type == "SOLID_STATE":
        if int(emitters) != 20_000 or int(report_rate) != 10 or not pattern.IsValid():
            raise RuntimeError(
                f"Invalid Petal Scan profile: emitters={emitters}, reportRate={report_rate}, "
                f"pattern={pattern.IsValid()}"
            )
        expected_pattern = {
            "encoding": PATTERN_ENCODING,
            "pointRateHz": 200_000,
            "scanRateHz": 10,
            "pointsPerState": 20_000,
            "trajectoryStates": 40,
        }
        for name, expected in expected_pattern.items():
            actual = pattern.GetAttribute(f"lidarHiking:{name}").Get()
            if actual != expected:
                raise RuntimeError(f"Unexpected pattern {name}: {actual!r} != {expected!r}")
        profile = "petal_scan"
    elif scan_type == "ROTARY":
        if int(emitters) != 40 or int(report_rate) != 5000 or pattern.IsValid():
            raise RuntimeError(
                f"Invalid Rotary Scan profile: emitters={emitters}, reportRate={report_rate}, "
                f"unexpectedPattern={pattern.IsValid()}"
            )
        profile = "rotary_scan"
    else:
        raise RuntimeError(f"Unsupported MID-360 scanType={scan_type!r}")

    mount = stage.GetPrimAtPath(MOUNT_PATH)
    if not mount.IsValid() or not mount.IsA(UsdGeom.Xformable):
        raise RuntimeError(f"Invalid Mid-360 mount at {MOUNT_PATH}")
    local = UsdGeom.Xformable(mount).GetLocalTransformation()
    sensor_up_in_parent = local.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)).GetNormalized()
    if sensor_up_in_parent[2] > -0.999:
        raise RuntimeError("Mid-360 mount is not upside down. Run fix_mid360_upside_down_isaacsim51.py first.")

    imu = stage.GetPrimAtPath(IMU_PATH)
    if not imu.IsValid() or imu.GetTypeName() != "IsaacImuSensor":
        raise RuntimeError(f"No matching Isaac IMU at {IMU_PATH}")
    imu_local = UsdGeom.Xformable(imu).GetLocalTransformation()
    max_pose_error = max(
        abs(float(local[row][column] - imu_local[row][column])) for row in range(4) for column in range(4)
    )
    if max_pose_error > 1.0e-6:
        raise RuntimeError(
            "Mid-360 LiDAR/IMU poses differ; identity sensor extrinsics would tilt the data "
            f"(max matrix error={max_pose_error:.3e})"
        )
    print(
        "[Mid360ROS2] LiDAR and IMU are colocated/aligned (identity extrinsics)"
    )
    print(f"[Mid360ROS2] Validated asset profile: {profile}")
    return profile


def _load_pattern_runtime(stage: Usd.Stage) -> dict:
    pattern = stage.GetPrimAtPath(PATTERN_PATH)
    compressed = bytes(pattern.GetAttribute("lidarHiking:compressedDirections").Get())
    raw = zlib.decompress(compressed)
    values = array.array("H")
    values.frombytes(raw)
    if sys.byteorder != "little":
        values.byteswap()

    points_per_state = int(pattern.GetAttribute("lidarHiking:pointsPerState").Get())
    trajectory_states = int(pattern.GetAttribute("lidarHiking:trajectoryStates").Get())
    if len(values) != points_per_state * trajectory_states * 2:
        raise RuntimeError(
            f"Corrupt MID-360 trajectory: {len(values)} uint16 values for "
            f"{points_per_state} points x {trajectory_states} states"
        )
    return {
        "values": values,
        "points_per_state": points_per_state,
        "trajectory_states": trajectory_states,
        "current_state": 0,
    }


def _apply_pattern_state(runtime: dict, state_index: int) -> None:
    if state_index == runtime["current_state"]:
        return
    points_per_state = runtime["points_per_state"]
    start = state_index * points_per_state * 2
    values = runtime["values"]
    azimuth_values = [values[start + index * 2] * 0.01 for index in range(points_per_state)]
    azimuth = Vt.FloatArray([value if value < 180.0 else value - 360.0 for value in azimuth_values])
    elevation = Vt.FloatArray(
        [values[start + index * 2 + 1] * 0.01 - 10.0 for index in range(points_per_state)]
    )
    stage = runtime["stage"]
    lidar = stage.GetPrimAtPath(LIDAR_PATH)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        lidar.GetAttribute(PATTERN_ATTRIBUTE_NAMES[0]).Set(azimuth)
        lidar.GetAttribute(PATTERN_ATTRIBUTE_NAMES[1]).Set(elevation)
    runtime["current_state"] = state_index


def _on_pattern_update(runtime: dict) -> None:
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        return
    state_index = int(timeline.get_current_time() * 10.0 + 1.0e-6) % runtime["trajectory_states"]
    _apply_pattern_state(runtime, state_index)


def _start_pattern_driver(stage: Usd.Stage) -> dict:
    runtime = _load_pattern_runtime(stage)
    runtime["stage"] = stage
    stream = omni.kit.app.get_app().get_update_event_stream()
    runtime["subscription"] = stream.create_subscription_to_pop(
        lambda _event: _on_pattern_update(runtime),
        name="MID-360 non-repetitive trajectory driver",
    )
    print(
        "[Mid360ROS2] Non-repetitive trajectory driver ready: "
        f"{runtime['trajectory_states']} x 0.1 s states, 200,000 points/s"
    )
    return runtime


def _stop_pattern_driver(runtime: dict) -> None:
    pattern_runtime = runtime.pop("pattern_runtime", None)
    if pattern_runtime is None:
        return
    pattern_runtime["subscription"] = None
    stage = pattern_runtime["stage"]
    layer = stage.GetSessionLayer()
    edits = Sdf.BatchNamespaceEdit()
    has_overrides = False
    for name in PATTERN_ATTRIBUTE_NAMES:
        property_path = Sdf.Path(LIDAR_PATH).AppendProperty(name)
        if layer.GetAttributeAtPath(property_path) is not None:
            edits.Add(Sdf.NamespaceEdit.Remove(property_path))
            has_overrides = True
    if has_overrides and not layer.Apply(edits):
        carb.log_warn("[Mid360ROS2] Could not remove trajectory session overrides")


def _require_ros2_bridge() -> None:
    manager = omni.kit.app.get_app().get_extension_manager()
    extension_id = "isaacsim.ros2.bridge"
    enabled = manager.is_extension_enabled(extension_id)
    required_nodes = (
        "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
        "isaacsim.ros2.bridge.ROS2PublishTransformTree",
    )
    versions = {node: og.GraphRegistry().get_node_type_version(node) or 0 for node in required_nodes}
    if not enabled or any(version < 1 for version in versions.values()):
        raise RuntimeError(
            "ROS 2 Bridge is not active. Restart Isaac Sim 5.1 with the "
            "isaacsim.ros2.bridge extension enabled, reopen the G1 USD, and "
            "run this publisher again."
        )
    print("[Mid360ROS2] ROS 2 Bridge ready; LiDAR helper and TF publisher registered")


def _require_gpu_lidar_output() -> None:
    setting = "/app/sensors/nv/lidar/outputBufferOnGPU"
    output_on_gpu = bool(carb.settings.get_settings().get(setting))
    if not output_on_gpu:
        raise RuntimeError(
            f"{setting}=false. This Isaac Sim 5.1 build crashed while copying "
            "the CPU RTX LiDAR buffer. Restart Isaac Sim with GPU LiDAR output "
            "enabled before publishing."
        )
    print("[Mid360ROS2] LiDAR renderer output buffer: GPU (Isaac Sim 5.1 path)")


def _unify_existing_scene_publishers(stage: Usd.Stage) -> None:
    """Put known optional scene publishers in the same simulation-time domain."""

    for graph_path, publisher_names in SIMULATION_TIME_CONTRACTS:
        source_text = graph_path + "/ReadSimulationTime.outputs:simulationTime"
        source_path = Sdf.Path(source_text)
        source = stage.GetAttributeAtPath(source_path)
        if not source.IsValid():
            continue

        reset_path = Sdf.Path(graph_path + "/ReadSimulationTime.inputs:resetOnStop")
        reset_on_stop = stage.GetAttributeAtPath(reset_path)
        if not reset_on_stop.IsValid():
            raise RuntimeError(f"Missing simulation-time reset input {reset_path}")

        with Usd.EditContext(stage, stage.GetSessionLayer()):
            if not reset_on_stop.Set(False):
                raise RuntimeError(f"Could not disable simulation-time reset at {reset_path}")
            for publisher_name in publisher_names:
                input_text = graph_path + f"/{publisher_name}.inputs:timeStamp"
                input_path = Sdf.Path(input_text)
                timestamp_input = stage.GetAttributeAtPath(input_path)
                if not timestamp_input.IsValid():
                    raise RuntimeError(f"Missing ROS timestamp input {input_path}")
                if not timestamp_input.SetConnections([source_path]):
                    raise RuntimeError(f"Could not connect {input_path} to {source_text}")

        for publisher_name in publisher_names:
            input_text = graph_path + f"/{publisher_name}.inputs:timeStamp"
            timestamp_input = stage.GetAttributeAtPath(Sdf.Path(input_text))
            connections = [str(path) for path in timestamp_input.GetConnections()]
            if connections != [source_text]:
                raise RuntimeError(f"ROS timestamp contract failed at {input_text}: {connections}")
        if bool(reset_on_stop.Get()):
            raise RuntimeError(f"Simulation time would reset when {graph_path} stops")

    print("[Mid360ROS2] Existing scene publishers use non-resetting simulation time")


def _find_test_runtime() -> dict | None:
    """Find the smoke-test runtime whether Script Editor used __main__ or globals."""

    candidates = (
        globals().get("_MID360_TEST_RUNTIME"),
        getattr(__main__, "_MID360_TEST_RUNTIME", None),
    )
    for runtime in candidates:
        if isinstance(runtime, dict):
            return runtime
    return None


def _get_live_render_product(stage: Usd.Stage) -> tuple[str, object | None, bool]:
    test_runtime = _find_test_runtime()
    if test_runtime is not None:
        raise RuntimeError(
            "Mid360 point-cloud test is still active. Run "
            "cleanup_mid360_test(), wait for its cleanup message, then run "
            "this publisher again. The ROS publisher will create its own "
            "render product."
        )

    handle = rep.create.render_product(
        LIDAR_PATH,
        resolution=(32, 32),
        name="Mid360ROS2RenderProduct",
        render_vars=["GenericModelOutput", "RtxSensorMetadata"],
    )
    print(f"[Mid360ROS2] Created LIVE LiDAR render product: {handle.path}")
    return handle.path, handle, True


def _disable_gravity_in_session(stage: Usd.Stage) -> Sdf.Path | None:
    """Retain normal gravity in the loaded scene."""

    del stage
    print("[Mid360ROS2] Scene gravity remains enabled")
    return None


def _restore_gravity(runtime: dict) -> None:
    stage = runtime.get("stage")
    scene_path = runtime.get("gravity_scene_path")
    if stage is None or scene_path is None:
        return
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        scene_prim = stage.GetPrimAtPath(scene_path)
        if scene_prim.IsValid():
            scene_prim.RemoveProperty("physics:gravityMagnitude")


def _remove_graph(stage: Usd.Stage, graph_path_text: str) -> None:
    session_layer = stage.GetSessionLayer()
    graph_path = Sdf.Path(graph_path_text)
    graph = stage.GetPrimAtPath(graph_path)
    if graph.IsValid():
        unexpected = [
            spec.layer.identifier for spec in graph.GetPrimStack() if spec.layer.identifier != session_layer.identifier
        ]
        if unexpected:
            raise RuntimeError(
                f"Refusing to remove {graph_path_text}; it is authored outside "
                "the runtime session layer: "
                f"unexpected layers: {unexpected}"
            )

    _remove_spec(session_layer, graph_path)
    if stage.GetPrimAtPath(graph_path).IsValid():
        raise RuntimeError(f"Could not completely remove runtime Action Graph {graph_path_text}")


def _move_graph_to_session(stage: Usd.Stage, graph_path_text: str) -> None:
    """Keep Controller-created graph specs out of the persistent robot USD."""

    graph_path = Sdf.Path(graph_path_text)
    root_layer = stage.GetRootLayer()
    session_layer = stage.GetSessionLayer()
    if root_layer.GetPrimAtPath(graph_path) is None:
        return

    # In Isaac Sim 5.1, og.Controller.edit may create its graph in the root
    # layer despite the surrounding Usd.EditContext.  Copy the complete graph
    # subtree to the stronger session layer before removing the root opinion.
    _remove_spec(session_layer, graph_path)
    with Sdf.ChangeBlock():
        if not Sdf.CopySpec(root_layer, graph_path, session_layer, graph_path):
            raise RuntimeError(f"Could not copy {graph_path_text} from root to session layer")
        _remove_spec(root_layer, graph_path)

    # The graph is now absent from the persistent layer's in-memory content.
    # Do not save the scene automatically; user scene edits remain under user
    # control, while the graph stays reusable and runtime-only.
    print(f"[Mid360ROS2] Action Graph isolated in Session Layer: {graph_path_text}")


def _cleanup_runtime(runtime: dict, *, stop_timeline: bool) -> None:
    if stop_timeline:
        omni.timeline.get_timeline_interface().stop()
    _clear_selection()
    _stop_pattern_driver(runtime)

    stage = runtime.get("stage")
    if stage is not None:
        _remove_graph(stage, runtime["graph_path"])

    if runtime.get("owns_render_product"):
        handle = runtime.get("render_product_handle")
        if handle is not None:
            try:
                handle.destroy()
            except Exception as error:
                carb.log_warn(f"[Mid360ROS2] Render-product cleanup warning: {error}")
    _restore_gravity(runtime)


def cleanup_mid360_ros2() -> None:
    """Stop publishing and remove resources owned by this publisher script."""

    runtime = globals().pop("_MID360_ROS2_RUNTIME", None)
    if runtime is None:
        runtime = getattr(__main__, "_MID360_ROS2_RUNTIME", None)
        if runtime is not None:
            delattr(__main__, "_MID360_ROS2_RUNTIME")
    if runtime is None:
        print("[Mid360ROS2] Nothing to clean up")
        return
    _cleanup_runtime(runtime, stop_timeline=True)
    print("[Mid360ROS2] Publisher stopped; temporary Action Graph removed")


def _build_action_graph(stage: Usd.Stage, render_product_path: str) -> None:
    session_layer = stage.GetSessionLayer()
    _remove_graph(stage, GRAPH_PATH)

    with Usd.EditContext(stage, session_layer):
        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    (
                        "ReadSimulationTime",
                        "isaacsim.core.nodes.IsaacReadSimulationTime",
                    ),
                    (
                        "PublishPointCloud",
                        "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
                    ),
                    (
                        "PublishMountTF",
                        "isaacsim.ros2.bridge.ROS2PublishTransformTree",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (
                        "PublishPointCloud.inputs:renderProductPath",
                        render_product_path,
                    ),
                    ("PublishPointCloud.inputs:topicName", TOPIC_NAME),
                    ("PublishPointCloud.inputs:frameId", FRAME_ID),
                    ("PublishPointCloud.inputs:type", "point_cloud"),
                    ("PublishPointCloud.inputs:fullScan", PUBLISH_FULL_SCAN),
                    (
                        "PublishPointCloud.inputs:frameSkipCount",
                        PUBLISH_FRAME_SKIP_COUNT,
                    ),
                    ("PublishPointCloud.inputs:queueSize", QUEUE_SIZE),
                    ("PublishPointCloud.inputs:showDebugView", False),
                    ("PublishPointCloud.inputs:useSystemTime", USE_SYSTEM_TIME),
                    (
                        "PublishPointCloud.inputs:resetSimulationTimeOnStop",
                        False,
                    ),
                    ("PublishMountTF.inputs:topicName", "tf"),
                    (
                        "PublishMountTF.inputs:parentPrim",
                        [usdrt.Sdf.Path(ROBOT_ROOT_PATH)],
                    ),
                    (
                        "PublishMountTF.inputs:targetPrims",
                        [usdrt.Sdf.Path(MOUNT_PATH)],
                    ),
                    ("PublishMountTF.inputs:queueSize", QUEUE_SIZE),
                    ("PublishMountTF.inputs:staticPublisher", False),
                    ("ReadSimulationTime.inputs:resetOnStop", False),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        "OnPlaybackTick.outputs:tick",
                        "PublishPointCloud.inputs:execIn",
                    ),
                    (
                        "OnPlaybackTick.outputs:tick",
                        "PublishMountTF.inputs:execIn",
                    ),
                    (
                        "ReadSimulationTime.outputs:simulationTime",
                        "PublishMountTF.inputs:timeStamp",
                    ),
                ],
            },
        )

    _move_graph_to_session(stage, GRAPH_PATH)

    if not stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        raise RuntimeError(f"Action Graph creation failed at {GRAPH_PATH}")

    graph_layers = {spec.layer.identifier for spec in stage.GetPrimAtPath(GRAPH_PATH).GetPrimStack()}
    if graph_layers != {session_layer.identifier}:
        raise RuntimeError(f"Action Graph is not session-only; composed layers: {graph_layers}")


def main() -> None:
    _clear_selection()
    previous = globals().pop("_MID360_ROS2_RUNTIME", None)
    if previous is None:
        previous = getattr(__main__, "_MID360_ROS2_RUNTIME", None)
        if previous is not None:
            delattr(__main__, "_MID360_ROS2_RUNTIME")
    if previous is not None:
        _cleanup_runtime(previous, stop_timeline=True)

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Open a scene containing a standardized G1 + MID-360 asset first")

    _resolve_robot_instance(stage)
    profile = _validate_lidar(stage)
    _require_ros2_bridge()
    _require_gpu_lidar_output()

    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        timeline.stop()
    _unify_existing_scene_publishers(stage)

    render_product_path, render_product_handle, owns_render_product = _get_live_render_product(stage)
    gravity_scene_path = None
    try:
        gravity_scene_path = _disable_gravity_in_session(stage)
        _build_action_graph(stage, render_product_path)
        pattern_runtime = _start_pattern_driver(stage) if profile == "petal_scan" else None
    except Exception:
        if owns_render_product and render_product_handle is not None:
            render_product_handle.destroy()
        _restore_gravity({"stage": stage, "gravity_scene_path": gravity_scene_path})
        raise

    runtime = {
        "stage": stage,
        "render_product_path": render_product_path,
        "render_product_handle": render_product_handle,
        "owns_render_product": owns_render_product,
        "gravity_scene_path": gravity_scene_path,
        "graph_path": GRAPH_PATH,
        "pattern_runtime": pattern_runtime,
    }
    globals()["_MID360_ROS2_RUNTIME"] = runtime
    vars(__main__)["_MID360_ROS2_RUNTIME"] = runtime
    globals()["cleanup_mid360_ros2"] = cleanup_mid360_ros2
    vars(__main__)["cleanup_mid360_ros2"] = cleanup_mid360_ros2

    print(f"[Mid360ROS2] Action Graph created: {GRAPH_PATH}")
    print(f"[Mid360ROS2] Publishing sensor_msgs/PointCloud2: /{TOPIC_NAME}")
    print(
        f"[Mid360ROS2] Publishing TF: {ROBOT_FIXED_FRAME_ID} -> {FRAME_ID}; "
        f"use Fixed Frame={ROBOT_FIXED_FRAME_ID} in RViz for an upright view"
    )
    print(
        f"[Mid360ROS2] frame_id={FRAME_ID}, fullScan={PUBLISH_FULL_SCAN}, "
        f"frameSkipCount={PUBLISH_FRAME_SKIP_COUNT}; safe per-render-frame "
        "publishing is active"
    )
    print("[Mid360ROS2] timestamp domain: /clock simulation time (useSystemTime=false)")
    print(
        "[Mid360ROS2] Topic rate follows render rate (not 10 Hz); accumulate "
        "0.1 s of slices downstream when a 10 Hz complete scan is required"
    )
    if AUTO_START_TIMELINE:
        timeline.play()
        print("[Mid360ROS2] Timeline PLAY")
    else:
        print(
            "[Mid360ROS2] Timeline remains paused. Start ROS 2 consumers, then "
            "press Play in Isaac Sim."
        )


main()
