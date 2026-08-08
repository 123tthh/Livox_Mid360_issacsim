# Livox MID360 for Isaac Sim

面向 NVIDIA Isaac Sim 5.1 的 Livox MID360 仿真资产，包含可直接引用的
Unitree G1 4010 与 5010 两套 29-DoF 机器人 USD，以及运行时 ROS 2
`sensor_msgs/PointCloud2` 发布脚本。

> 仓库名沿用 `issacsim`，产品名和文档统一写作 **Isaac Sim**。

## 仓库内容

| 资产 | G1 版本 | 电机/模式 | USD |
| --- | --- | --- | --- |
| 4010 基线 | `g1_29dof_rev_1_0` | wrist 4010，mode_machine 5 | `assets/g1_29dof_rev_1_0/G1_29dof_mid360.usd` |
| 5010 当前版 | `g1_29dof_mode_13` | wrist 5010，mode_machine 13 | `assets/g1_29dof_mode_13_5010/G1_29dof_mode_13_5010_mid360.usd` |

两份 USD 都是展平后的单文件资产，没有外部 USD 引用，也没有把 ROS、机器人状态和运动控制
Action Graph 固化进资产。点云发布图由 `scripts/publish_mid360_ros2_isaacsim51.py`
在 Session Layer 中独立创建，便于修改且不会污染 USD。

## 当前 MID360 配置

这是针对机器人地形感知配置的 **40 线旋转近似模型**，不是 Livox 真机的非重复花瓣扫描轨迹。
4010 与 5010 使用相同的传感器合同：

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| USD 类型 | `OmniLidar` | Isaac Sim RTX LiDAR |
| `scanType` | `ROTARY` | 旋转扫描近似 |
| `numberOfEmitters` | 40 | 40 条垂直发射线 |
| `scanRateBaseHz` | 10 Hz | 一圈/完整扫描周期 |
| 每线 report rate | 5000 Hz | 总目标点率约 `40 × 5000 = 200,000 points/s` |
| 传感器坐标系俯仰 | -7° 到 +52° | 发射线覆盖范围 |
| 安装滚转 | 180° | 倒装后机器人坐标系覆盖约 -52° 到 +7° |
| 5010 安装位移 | `[0.0002835, 0.00003, 0.41618]` m | 相对 `torso_link` |
| 最小/最大量程 | 0.1 m / 70 m | 10% 反射率保证距离按 40 m 记录 |
| 点云话题 | `/mid360/points` | `sensor_msgs/PointCloud2` |
| 点云 frame | `mid360_link` | 同时发布机器人根到雷达安装座的 `/tf` |
| 时间 | Isaac simulation time | `useSystemTime=false`，停止后不重置 |
| ROS 发布 | 每个渲染帧，不跳帧 | `fullScan=false`，避免 Isaac Sim 5.1 累积全扫描崩溃路径 |

如果 FAST-LIO 需要 10 Hz 完整扫描，应在下游按 **0.1 s** 聚合所有渲染帧切片；不要通过
丢帧或仅发布 0.47 倍点云来降载。LiDAR 与配套仿真 IMU 在资产中同位同向，因此该仿真合同
要求 FAST-LIO 使用 `extrinsic_T: [0, 0, 0]`、`extrinsic_R: I`，不能直接套用真机
MID360 标定外参。

## 修改 MID360 参数

推荐复制资产后修改，并同步更新对应 `manifest.json`：

1. 在 Isaac Sim Stage 中展开机器人，选择
   `torso_link/mid360_link/mid360_native_approx`。
2. 在 Property 面板的 Omni Sensor Generic LiDAR API 中修改：
   - 扫描频率：`omni:sensor:Core:scanRateBaseHz`；
   - 发射线数量：`omni:sensor:Core:numberOfEmitters`；
   - 扫描类型：`omni:sensor:Core:scanType`；
   - 每线 report rate、各 emitter 的方位/俯仰角、量程和反射率参数。
3. 修改安装位置或方向时，选择父节点 `mid360_link`；LiDAR 与
   `mid360_imu` 必须应用完全相同的变换，否则 FAST-LIO 地图会倾斜或重叠。
4. 若改 `numberOfEmitters`，必须同时重建相同数量的 emitter-state API 实例；只改计数会产生
   无效配置。若改扫描频率或渲染帧率，也要同步修改下游累积窗口，保证统一时间戳。
5. 保存为新文件并运行 `python scripts/validate_assets.py`，不要直接覆盖已发布的基准资产。

5010 可通过 `scripts/build_g1_5010_mid360_asset.py` 重建。脚本从 4010 USD 复制
`mid360_link`，因此先修改 4010 传感器源，再重建 5010，可保证两版本参数一致。

## 使用

1. 使用 Isaac Sim 5.1 打开任一 USD，启用 `isaacsim.ros2.bridge`，并确保
   `/app/sensors/nv/lidar/outputBufferOnGPU=true`。
2. 打开 **Window → Script Editor**，载入并运行
   `scripts/publish_mid360_ros2_isaacsim51.py`。
3. 启动 ROS 2 消费端后按 Play。脚本默认保持时间线暂停，不会抢占运动控制。
4. 清理临时发布图：在 Script Editor 执行 `cleanup_mid360_ros2()`。

场景中有多个 G1 时，在运行发布脚本前设置：

```python
MID360_ROBOT_ROOT_PATH = "/World/G1_A"
```

验证文件完整性：

```bash
python3 scripts/validate_assets.py
python3 -m unittest discover -s tests -v
```

将两套资产安装到 Lidar_Hiking 工程的兼容路径：

```bash
python3 scripts/install_into_lidar_hiking.py /path/to/Lidar_Hiking
```

## 限制

- 本仓库只提供传感器、机器人组合资产和 ROS 2 发布，不包含 Lidar_Hiking 策略、训练场景或
  FAST-LIO 工程。
- 40 线旋转扫描用于可重复仿真和策略开发，不能用于评估真实 MID360 扫描花纹误差。
- `fullScan=true` 在已验证的 Isaac Sim 5.1.0-rc.19 环境中不稳定，默认保持关闭。

## 来源与许可证

本仓库原创脚本使用根目录 MIT License。Unitree G1、InstinctLab 训练碰撞模型及派生资产仍受
各自许可证约束，详见 `THIRD_PARTY_NOTICES.md` 和资产目录内保留的许可证。
