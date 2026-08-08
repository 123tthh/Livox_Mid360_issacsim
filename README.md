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

4010 与 5010 现在使用相同的 **非重复花瓣扫描轨迹**。轨迹来自 Livox 官方
`Livox-SDK/livox_laser_simulation` 的 `mid360.csv`；800,000 条有序方向构成
4 秒参考窗口。为满足 Isaac Sim 5.1 单传感器 5 MiB 属性上限，完整轨迹压缩存入 USD 内的
兄弟 Scope，`OmniLidar` 保持一个固定长度 state，ROS 运行脚本按 10 Hz 只替换数组值：

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| USD 类型 | `OmniLidar` | Isaac Sim RTX LiDAR |
| `scanType` | `SOLID_STATE` | Livox 旋转镜混合固态扫描 |
| 轨迹状态 | 40 个 | 每个状态对应连续 0.1 s 真机轨迹 |
| RTX emitter state | 1 个 | 固定数组长度，运行期 10 Hz 换值，规避 5 MiB 上限 |
| `numberOfEmitters` | 20,000/状态 | 10 Hz × 20,000 = 200,000 points/s |
| `numberOfChannels` | 20,000 | Core 固态模型中每条独立定时射线映射一个通道；官方 CSV 不含物理激光器 ID |
| `scanRateBaseHz` / report rate | 10 Hz / 10 Hz | 每个状态输出一个 0.1 s 点云帧 |
| 单点发射时序 | 5 μs | 每帧 `0 … 99.995 ms`，支持运动畸变 |
| 传感器坐标系标称俯仰 | -7° 到 +52° | 官方规格的 59° 竖直 FOV |
| 参考轨迹实际俯仰 | 约 -7.2123° 到 +52.164° | 官方表格的亚角度边缘摆动 |
| 安装滚转 | 180° | 倒装后机器人坐标系覆盖约 -52° 到 +7° |
| 5010 安装位移 | `[0.0002835, 0.00003, 0.41618]` m | 相对 `torso_link` |
| 最小/最大量程 | 0.1 m / 70 m | 10% 反射率保证距离按 40 m 记录 |
| 点云话题 | `/mid360/points` | `sensor_msgs/PointCloud2` |
| 点云 frame | `mid360_link` | 同时发布机器人根到雷达安装座的 `/tf` |
| 时间 | Isaac simulation time | `useSystemTime=false`，停止后不重置 |
| ROS 发布 | 每个渲染帧，不跳帧 | `fullScan=false`，避免 Isaac Sim 5.1 累积全扫描崩溃路径 |

Isaac Sim 5.1 的 ROS 2 helper 仍使用稳定的 `fullScan=false` 路径，因此 FAST-LIO 应在下游按
**0.1 s** 聚合所有渲染帧切片；不要通过
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
4. RTX state 的 `azimuthDeg`、`elevationDeg`、`fireTimeNs`、`channelId` 和 `bank`
   必须各有 20,000 项；完整 40-state 压缩轨迹由生成器维护，不能直接删除兄弟 Scope。
5. 修改扫描频率或渲染帧率时，要同步修改逐点时间和下游累积窗口，保证统一时间戳。
6. 保存为新文件并运行 `python scripts/validate_assets.py`，不要直接覆盖已发布的基准资产。

用 Livox 官方轨迹重新写入两份资产：

```bash
/path/to/isaac-sim/python.sh scripts/apply_mid360_nonrepetitive_profile.py
```

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

安装器还会同步组合场景使用的 Isaac Sim 5.1 ROS 2 发布脚本，使 40-state
轨迹驱动器随 `/mid360/points` 发布自动启动。

## 限制

- 本仓库只提供传感器、机器人组合资产和 ROS 2 发布，不包含 Lidar_Hiking 策略、训练场景或
  FAST-LIO 工程。
- 轨迹是 Livox 官方仿真仓库提供的 4 秒参考窗口，窗口内非重复，但状态序列在第 4 秒后循环；
  它用于逼近空间覆盖和时序，不替代逐台真机光学校准。
- `fullScan=true` 在已验证的 Isaac Sim 5.1.0-rc.19 环境中不稳定，默认保持关闭。

## 来源与许可证

本仓库原创脚本使用根目录 MIT License。Unitree G1、InstinctLab 训练碰撞模型及派生资产仍受
各自许可证约束，详见 `THIRD_PARTY_NOTICES.md` 和资产目录内保留的许可证。
