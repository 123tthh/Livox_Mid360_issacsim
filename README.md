# Livox MID-360 for Isaac Sim 5.1

项目规范名：**Livox_MID360_IsaacSim**。

本仓库提供两套明确分离的 MID-360 仿真扫描配置，每套都包含独立雷达、
Unitree G1 4010 含雷达机器人和 Unitree G1 5010 含雷达机器人，共六份可发布资产。

## 资产矩阵

| 扫描配置 | 独立 MID-360 | G1 4010 + MID-360 | G1 5010 + MID-360 |
| --- | --- | --- | --- |
| Petal Scan（花瓣扫描） | `assets/petal_scan/standalone/Livox_MID360_Petal_Scan.usd` | `assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd` | `assets/petal_scan/g1_5010/Unitree_G1_5010_MID360_Petal_Scan.usd` |
| Rotary Scan（普通旋转近似） | `assets/rotary_scan/standalone/Livox_MID360_Rotary_Scan.usd` | `assets/rotary_scan/g1_4010/Unitree_G1_4010_MID360_Rotary_Scan.usd` | `assets/rotary_scan/g1_5010/Unitree_G1_5010_MID360_Rotary_Scan.usd` |

共用资源位于 `assets/common/`：

- `Livox_MID360_CAD.usd`：由用户提供的 STEP 装配转换得到的 CAD 几何；
- `mid360_official_pattern/mid360.csv`：Livox 官方四秒参考轨迹；
- `g1_5010_support/`：5010 机器人来源、网格和训练碰撞 URDF。

## Petal Scan（花瓣扫描）

Petal Scan 使用 Livox 官方 `livox_laser_simulation` 的 800,000 条有序方向，
在四秒参考窗口内形成非重复花瓣覆盖：

| 参数 | 值 |
| --- | --- |
| `scanType` | `SOLID_STATE` |
| 输出点率 | 200,000 points/s |
| 扫描帧率 | 10 Hz |
| 轨迹状态 | 40 × 0.1 s |
| 每状态射线 | 20,000 |
| 单点时序 | 5 μs |
| 参考窗口 | 4 s；之后循环 |
| 标称垂直 FOV | -7° 到 +52° |

Isaac Sim 5.1 对单传感器属性有大小限制，因此 USD 内的 OmniLidar 始终保留一个
20,000-ray RTX state，运行驱动器每 0.1 秒替换同长度数组。完整轨迹压缩保存在雷达旁的
`mid360_nonrepetitive_pattern` Scope 中。

## Rotary Scan（普通雷达）

Rotary Scan 保留原项目的确定性 40 线旋转近似，适用于调试、性能基线和不需要真机花纹的场景：

| 参数 | 值 |
| --- | --- |
| `scanType` | `ROTARY` |
| emitters | 40 |
| 每线 report rate | 5,000 Hz |
| 扫描频率 | 10 Hz |
| 目标点率 | 40 × 5,000 = 200,000 points/s |
| 垂直覆盖 | -7° 到 +52° |

Rotary Scan 不包含花瓣轨迹 Scope，也不启动运行时状态切换器。

## 机器人安装合同

4010 和 5010 的雷达安装保持一致：

- 相对 `torso_link` 平移：`[0.0002835, 0.00003, 0.41618]` m；
- 滚转：180° 倒装；
- LiDAR 和 `mid360_imu` 同位同向；
- IMU 类型：`IsaacImuSensor`，周期 `0.005 s`（200 Hz）；
- ROS 点云 frame：`mid360_link`；
- ROS IMU 话题：`/mid360/imu`，frame：`mid360_link`；
- TF：机器人根节点到 `mid360_link`；
- FAST-LIO 仿真外参：`extrinsic_T=[0,0,0]`、`extrinsic_R=I`。

两份 G1 USD 都是展平的单文件机器人资产，不固化运动策略、建图或控制 Action Graph。

## Isaac Sim 5.1 使用

1. 直接打开或引用资产矩阵中的任意 USD。
2. 启用 `isaacsim.ros2.bridge`，并设置
   `/app/sensors/nv/lidar/outputBufferOnGPU=true`。
3. 在 Script Editor 运行 `scripts/publish_mid360_ros2_isaacsim51.py`。
4. Petal Scan 会自动启动 40 状态驱动；Rotary Scan 直接使用 USD 内的固定旋转配置。
5. 清理运行时图：调用 `cleanup_mid360_ros2()`。

读取 IMU 时，将各资产 README 中列出的 `mid360_imu` Prim 接入
`IsaacReadIMU` 和 `ROS2PublishImu`。六个资产各自目录中的 `README.md`
分别给出了 Prim 路径、ROS 2 话题和 FAST-LIO 外参设置。

独立资产会相对引用 `assets/common/Livox_MID360_CAD.usd`，克隆或复制时应保留仓库目录结构。

## 重建与验证

重新写入 Petal Scan 机器人配置：

```bash
/path/to/isaac-sim/python.sh scripts/apply_mid360_petal_profile.py
```

重建独立资产：

```bash
/path/to/isaac-sim/python.sh scripts/build_mid360_standalone_assets.py \
  assets/common/Livox_MID360_CAD.usd \
  assets/petal_scan/standalone/Livox_MID360_Petal_Scan.usd \
  --sensor-source assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd

/path/to/isaac-sim/python.sh scripts/build_mid360_standalone_assets.py \
  assets/common/Livox_MID360_CAD.usd \
  assets/rotary_scan/standalone/Livox_MID360_Rotary_Scan.usd \
  --sensor-source assets/rotary_scan/g1_4010/Unitree_G1_4010_MID360_Rotary_Scan.usd
```

重建后补入/刷新六个资产的 IMU：

```bash
/path/to/isaac-sim/python.sh scripts/add_mid360_imu_isaacsim51.py
```

同步清单并运行测试：

```bash
python3 scripts/update_asset_metadata.py
make check
```

## 限制

- Petal Scan 在官方四秒参考窗口内非重复，第 4 秒后循环，不等同于无限时长真机光学模型。
- Rotary Scan 是可重复的 40 线近似，不用于评估真实 MID-360 花瓣覆盖误差。
- `fullScan=true` 在验证使用的 Isaac Sim 5.1.0-rc.19 中不稳定，ROS 发布脚本保持
  `fullScan=false`，需要完整 10 Hz 扫描时应在下游聚合 0.1 秒切片。
- 仓库只提供传感器与机器人组合资产，不包含 Lidar_Hiking 策略或 FAST-LIO 工程。

## 来源与许可证

原创脚本使用根目录 MIT License。Livox 轨迹、Unitree G1、InstinctLab 派生资源及 CAD
来源说明见 `THIRD_PARTY_NOTICES.md` 和 `assets/common/` 内保留的许可证。
