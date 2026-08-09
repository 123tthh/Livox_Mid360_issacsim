# Livox MID-360 standalone asset for Isaac Sim 5.1

Import or reference `Livox_MID360_Petal_Scan.usd`. It contains:

- the MID-360 CAD assembly from `mid-360-asm.stp` (22 source meshes, 77,565 triangles);
- an Isaac Sim 5.1 `OmniLidar` configured for 200,000 points/s and 10 Hz;
- all 800,000 directions from the official Livox four-second non-repetitive scan table.

Keep the repository structure because the wrapper references
`../../common/Livox_MID360_CAD.usd` relatively.
The wrapper stage uses metres and Z-up; the CAD millimetre coordinates are
scaled by 0.001. Its default prim is `/MID360`, and the sensor is at
`/MID360/torso_link/mid360_link/mid360_native_approx`.

To animate all 40 scan states, open the USD in Isaac Sim 5.1, run the entire
`drive_mid360_petal_scan_isaacsim51.py` file from Window > Script Editor,
then press Play. Call `cleanup_mid360_pattern_driver()` to stop it.

The scan table is redistributed under the Livox SDK2 BSD-3-Clause license; see
the source repository's
`assets/common/mid360_official_pattern/LICENSE.livox_laser_simulation`.
The CAD geometry is derived from the repository owner's supplied STEP assembly
and is included here as the standalone Isaac Sim import asset requested for
this project.

## IMU 使用

独立资产内置 `/MID360/torso_link/mid360_imu`，类型为 `IsaacImuSensor`，
周期 `0.005 s`（200 Hz）。独立组件的 LiDAR 与 IMU 都位于组件原点且方向一致；
把 `/MID360` 安装到机器人时，两者会随组件一起变换。

ROS 2 中将该 Prim 接到 `IsaacReadIMU` 和 `ROS2PublishImu`，使用话题
`/mid360/imu` 与 `frame_id=mid360_link`。点云与 IMU 已同系，FAST-LIO 使用单位外参。
