"""Drive an embedded MID-360 trajectory in an already-open Isaac Sim stage.

Run this complete file in Window > Script Editor.  Set ``MID360_LIDAR_PATH``
in Script Editor first only when the stage contains more than one MID-360.
"""

from __future__ import annotations

import array
import sys
import zlib

import __main__
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Sdf, Usd, Vt

_ATTRIBUTES = (
    "omni:sensor:Core:emitterState:s001:azimuthDeg",
    "omni:sensor:Core:emitterState:s001:elevationDeg",
)


def cleanup_mid360_pattern_driver() -> None:
    runtime = getattr(__main__, "_MID360_PATTERN_DRIVER", None)
    if not isinstance(runtime, dict):
        return
    runtime["subscription"] = None
    layer = runtime["stage"].GetSessionLayer()
    edits = Sdf.BatchNamespaceEdit()
    changed = False
    for name in _ATTRIBUTES:
        path = Sdf.Path(runtime["lidar_path"]).AppendProperty(name)
        if layer.GetAttributeAtPath(path) is not None:
            edits.Add(Sdf.NamespaceEdit.Remove(path))
            changed = True
    if changed:
        layer.Apply(edits)
    delattr(__main__, "_MID360_PATTERN_DRIVER")
    print("[MID360] Non-repetitive trajectory driver stopped")


def _find(stage: Usd.Stage) -> tuple[str, Usd.Prim]:
    hint = getattr(__main__, "MID360_LIDAR_PATH", None)
    candidates = [stage.GetPrimAtPath(str(hint))] if hint else [
        prim
        for prim in stage.TraverseAll()
        if prim.GetTypeName() == "OmniLidar"
        and prim.GetParent().GetChild("mid360_nonrepetitive_pattern").IsValid()
    ]
    candidates = [prim for prim in candidates if prim.IsValid()]
    if len(candidates) != 1:
        raise RuntimeError(
            f"Expected one MID-360 OmniLidar, found {len(candidates)}; set MID360_LIDAR_PATH explicitly"
        )
    lidar = candidates[0]
    return str(lidar.GetPath()), lidar.GetParent().GetChild("mid360_nonrepetitive_pattern")


def _apply(runtime: dict, state: int) -> None:
    if runtime["current_state"] == state:
        return
    count = runtime["points_per_state"]
    start = state * count * 2
    values = runtime["values"]
    azimuth_unsigned = [values[start + i * 2] * 0.01 for i in range(count)]
    azimuth = Vt.FloatArray([v if v < 180.0 else v - 360.0 for v in azimuth_unsigned])
    elevation = Vt.FloatArray([values[start + i * 2 + 1] * 0.01 - 10.0 for i in range(count)])
    lidar = runtime["stage"].GetPrimAtPath(runtime["lidar_path"])
    with Usd.EditContext(runtime["stage"], runtime["stage"].GetSessionLayer()):
        lidar.GetAttribute(_ATTRIBUTES[0]).Set(azimuth)
        lidar.GetAttribute(_ATTRIBUTES[1]).Set(elevation)
    runtime["current_state"] = state


def _update(runtime: dict) -> None:
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        _apply(runtime, int(timeline.get_current_time() * 10.0 + 1.0e-6) % runtime["states"])


cleanup_mid360_pattern_driver()
stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("Open a USD stage before starting the MID-360 driver")
lidar_path, pattern = _find(stage)
compressed = bytes(pattern.GetAttribute("lidarHiking:compressedDirections").Get())
values = array.array("H")
values.frombytes(zlib.decompress(compressed))
if sys.byteorder != "little":
    values.byteswap()
runtime = {
    "stage": stage,
    "lidar_path": lidar_path,
    "values": values,
    "points_per_state": int(pattern.GetAttribute("lidarHiking:pointsPerState").Get()),
    "states": int(pattern.GetAttribute("lidarHiking:trajectoryStates").Get()),
    "current_state": 0,
}
runtime["subscription"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    lambda _event: _update(runtime), name="MID-360 non-repetitive pattern driver"
)
__main__._MID360_PATTERN_DRIVER = runtime
print(f"[MID360] Driving {runtime['states']} non-repeating 0.1 s states at {lidar_path}")
