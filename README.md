# CAPO-LeggedRobotOdometry 🦾  [![License](https://img.shields.io/badge/License-See%20LICENSE-blue.svg)](LICENSE)

**Language / 语言：** **English** | [中文](#capo-leggedrobotodometry--中文)

**CAPO-LeggedRobotOdometry** is a **pure proprioceptive odometry library** for legged robots, implemented with a **portable C++ estimator core** that depends only on **IMU and joint motor data**.

The core estimation logic is implemented in **`FusionEstimator/fusion_estimator.h`**. The file **`fusion_estimator_node.cpp`** provides a **ROS 2 wrapper** around this estimator, while the **`Matlab/`** folder contains examples for **MATLAB + C++ mixed compilation** and offline evaluation.

For side-by-side comparison, **`Matlab/Comparison/invariant-ekf/`** provides a MATLAB mixed-compilation workflow for **`invariant-ekf`**, making it easier to compare this repository against a representative open-source legged odometry baseline.

In 3D closed-loop trials (a **200 m** horizontal and **15 m** vertical loop), Astrall point-foot robot A achieves **0.1638 m** horizontal error and **0.219 m** vertical error; for wheel-legged robot B, the corresponding errors are **0.2264 m** and **0.199 m**.

<img src="https://i0.hdslb.com/bfs/new_dyn/a922a574770c0e9f1c6266fdf5c26a9133671525.jpg"
     alt="WeChat group QR code"
     width="260">
     
Welcome to join the wechat group
---

## 📄 Paper

**Contact-Anchored Proprioceptive Odometry for Quadruped Robots** (arXiv:2602.17393)

* Paper: https://arxiv.org/abs/2602.17393

If you use this repository in research, please consider citing the paper.

---

## 📦 Data Sharing (Go2-EDU ROS bags)

To help readers quickly validate the pipeline, we provide **Go2-EDU trial datasets**, including **real-world videos**, derived CSV files, and the corresponding **ROS bag topics/messages** required by this node, enabling fast reproduction and sanity checks.

* Download (GitHub Releases): https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry/releases/tag/DataForTest
* Recommended assets in that release include `GO2Flat`, `GO2Stairs`, `MPXY150Z10`, `MWXY150Z10`, `robot_flat_1_compress.zip`, and `robot_stairs_1_compress.zip`

> Note: the IMU on this Go2-EDU platform exhibits **noticeable yaw drift**, so the odometry accuracy is generally **worse than** the results reported for Astrall robots A and B in the paper.

---

## ✨ Key Features

| Category | Description |
| --- | --- |
| **Biped / Quadruped / Wheel-Legged Unified** | Online contact-set switching; stance legs are detected automatically, supporting fast transitions between standing and walking. |
| **IMU + Joint-Motor Only** | The estimator core works with only IMU and joint motor measurements, without requiring cameras or LiDAR. |
| **MATLAB / C++ Mixed Compilation** | The `Matlab/` folder provides MATLAB + MEX examples for calling the same C++ core, and `Matlab/Comparison/invariant-ekf/` includes a comparable mixed-compilation setup for `invariant-ekf`. |
| **Full 3D & Planar 2D** | Publishes both 6DoF odometry (`SMX/Odom`) and a gravity-flattened 2D odometry (`SMX/Odom_2D`). |
| **Portable Pure C++ Core** | The estimator core is isolated in `FusionEstimator/`, making it easier to reuse outside ROS2. |
| **Runtime Tuning** | Key parameters can be adjusted through `config.yaml`, and platform-dependent thresholds can be tuned for different robots. |

---

## 🏗️ Related Repositories

| Scope | Repository | Summary |
| --- | --- | --- |
| **Low-level / Drivers** | https://github.com/ShineMinxing/Ros2Go2Base | DDS bridge, Unitree SDK2 control, pointcloud→LaserScan, TF utilities |
| **Odometry** | **CAPO-LeggedRobotOdometry (this repo)** | Pure proprioceptive fusion, publishes `SMX/Odom` / `SMX/Odom_2D` |
| **SLAM / Mapping** | https://github.com/ShineMinxing/Ros2SLAM | Integrations for Cartographer 3D, KISS-ICP, FAST-LIO2, Point-LIO |
| **Voice / LLM** | https://github.com/ShineMinxing/Ros2Chat | Offline ASR + OpenAI Chat + TTS |
| **Vision** | https://github.com/ShineMinxing/Ros2ImageProcess | Camera pipelines, spot / face / drone detection |
| **Gimbal Tracking** | https://github.com/ShineMinxing/Ros2AmovG1 | Amov G1 gimbal control and tracking |
| **Tools** | https://github.com/ShineMinxing/Ros2Tools | Bluetooth IMU, joystick mapping, gimbal loop, data logging |

> ⚠️ **Clone as needed.** If you only need state estimation, this repository is sufficient. For mapping, it is natural to pair it with `Ros2SLAM` and `Ros2Go2Base`.

---

## 📂 Repository Layout

```text
CAPO-LeggedRobotOdometry/
├── CMakeLists.txt
├── package.xml
├── config.yaml
├── fusion_estimator_node.cpp        # ROS2 wrapper around the C++ estimator core
├── FusionEstimator/                 # portable pure C++ estimator core
│   ├── Estimators/
│   ├── fusion_estimator.h           # main estimator entry
│   ├── LowlevelState.h
│   ├── SensorBase.cpp
│   ├── SensorBase.h
│   ├── Sensor_IMU.cpp
│   ├── Sensor_IMU.h
│   ├── Sensor_Legs.cpp
│   ├── Sensor_Legs.h
│   └── Readme.md
├── Matlab/                          # MATLAB + MEX examples for the same C++ core
│   ├── build_mex.m
│   ├── fusion_estimator.m
│   ├── fusion_estimator_mex.cpp
│   ├── Comparison/
│   │   └── invariant-ekf/           # MATLAB mixed-compilation workflow for invariant-ekf
│   └── ...                          # optional test datasets are published via GitHub Releases
├── Plotjuggler.xml
└── Readme.md

```
---

## 🧩 Architecture Notes

This repository is intentionally split into three layers:

### 1. Estimator core
The actual odometry algorithm is implemented in **`FusionEstimator/fusion_estimator.h`** and the accompanying files under **`FusionEstimator/`**.  
This is the **main pure C++ proprioceptive estimator core**, designed to work with only **IMU and joint motor data**.

### 2. ROS2 wrapper
**`fusion_estimator_node.cpp`** wraps the estimator core into a ROS 2 node, handling:

* ROS2 topic subscriptions
* parameter loading
* message conversion
* odometry publication

### 3. MATLAB mixed-compilation support
The **`Matlab/`** folder provides examples for compiling and calling the same C++ estimator core from MATLAB via MEX.  
In addition, **`Matlab/Comparison/invariant-ekf/`** provides a comparable MATLAB mixed-compilation workflow for **`invariant-ekf`**, which is useful for offline benchmarking and side-by-side evaluation.

---

## ⚙️ Configuration (`config.yaml`)

The table below lists the main parameters used by the ROS2 node. See `config.yaml` for the full file and comments.

| Parameter | Typical Value | Meaning |
| --- | ---: | --- |
| `sub_imu_topic` | `SMX/Go2IMU` | IMU topic |
| `sub_joint_topic` | `SMX/Go2Joint` | joint state topic |
| `sub_mode_topic` | `SMX/SportCmd` | reset / mode topic |
| `pub_odom_topic` | `SMX/Odom` | 6DoF odometry output |
| `pub_odom2d_topic` | `SMX/Odom_2D` | planar odometry output |
| `odom_frame` | `odom` | odometry frame |
| `base_frame` | `base_link` | body frame |
| `base_frame_2d` | `base_link_2D` | planar body frame |
| `imu_data_enable` | `true` | enable IMU input |
| `leg_pos_enable` | `true` | enable leg-position kinematics |
| `leg_vel_enable` | `true` | enable leg-velocity kinematics |
| `leg_ori_enable` | `false` | enable kinematics-based yaw correction |
| `leg_ori_init_weight` | `0.001` | initial weight of orientation correction |
| `leg_ori_time_wight` | `1000.0` | time-related growth weight for orientation correction |
| `leg_velcke_enable` | `false` | enable IKVel-CKF / CAPO-CKE |
| `foot_force_threshold` | `20.0` | contact threshold |
| `min_stair_height` | `0.08` | minimum stair-height hypothesis used by the estimator |

> Notes:
> * The exact values are platform-dependent.
> * Some fields in `config.yaml` may be experimental, legacy, or intended for specific branches / workflows.
> * If you port only the pure C++ core, you usually need to keep only the estimator-relevant fields and can drop ROS2-specific naming.

---

## ⚙️ Build & Run

### Clone the repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --depth 1 https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry.git
cd ~/ros2_ws
colcon build --packages-select fusion_estimator
source install/setup.bash
ros2 run fusion_estimator fusion_estimator_node
```


## 📑 ROS 2 Interfaces

```text
/fusion_estimator_node (rclcpp)
├─ Publishes
│   • SMX/Odom         nav_msgs/Odometry (odom → base_link)
│   • SMX/Odom_2D      nav_msgs/Odometry (odom → base_link_2D)
├─ Subscribes
│   • SMX/Go2IMU       sensor_msgs/Imu
│   • SMX/Go2Joint     std_msgs/Float64MultiArray
│       layout:
│         data[0..11]   = 12 joint positions q
│         data[12..23]  = 12 joint velocities dq
│         data[24..27]  = 12 joint torque tau or contact-related values
│   • SMX/SportCmd     std_msgs/Float64MultiArray
│       reset example:
│         data[0] == 25140000  → estimator reset
└─ TF is typically published by another node / utility if needed
```

---

## 🧠 Algorithm Overview (High Level)

1. **Contact detection**  
   Detect stance legs from force / threshold logic.

2. **Forward kinematics**  
   Compute foot-end position / velocity in the body frame from joint measurements.

3. **Contact anchoring**  
   Record touchdown footfall points and use them as intermittent world-frame constraints during stance.

4. **Height stabilization**  
   Use support-plane height logic and confidence decay to reduce long-horizon elevation drift.

5. **Optional IKVel-CKF (CAPO-CKE)**  
   Suppress encoder-induced velocity spikes and obtain smoother leg-end velocity estimates.

6. **Optional yaw stabilization**  
   Use multi-contact geometric consistency to reduce IMU yaw drift during prolonged standing.

7. **2D projection**  
   Publish a planar odometry (`SMX/Odom_2D`) by flattening roll / pitch while preserving yaw and planar motion.

For full derivations, please refer to the paper.

---

## 🔧 Reusing `FusionEstimator/` Outside ROS2

If you do **not** need ROS2, the reusable part is mainly under `FusionEstimator/`.

A typical migration path is:

1. keep the estimator core in `FusionEstimator/`
2. replace ROS2 message subscriptions with your own sensor interface
3. prepare IMU, joint position / velocity, and contact / force inputs
4. call the estimator core from your own loop
5. export odometry to your target middleware / runtime

This design is useful for:

* ROS1 migration
* embedded deployment
* offline replay tools
* MATLAB / simulation analysis
* custom robotics middleware

---

## 🧪 MATLAB / MEX (Optional)

The **`Matlab/`** folder provides examples for compiling the estimator core into a MATLAB-callable MEX module.

Typical workflow:

```matlab
cd Matlab
build_mex
fusion_estimator
```

Files in this folder:

* `build_mex.m` — build script for MEX compilation
* `fusion_estimator_mex.cpp` — MEX bridge wrapping the C++ estimator core
* `fusion_estimator.m` — MATLAB-side usage / validation example
* sample CSV files and compressed test datasets — published via GitHub Releases: https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry/releases/tag/DataForTest

For comparison with another representative legged-odometry implementation, Matlab/Comparison/invariant-ekf/ also provides a MATLAB mixed-compilation workflow for invariant-ekf.

---

## 📈 Visualization

A `Plotjuggler.xml` file is included in the repository for convenient visualization and debugging with PlotJuggler.

You can use it to inspect:

* odometry outputs
* IMU-related signals
* joint / force-related channels
* estimator behavior during walking, standing, or reset events

---

## 🎥 Videos

| Topic | Link |
| --- | --- |
| Odometry-only mapping (morphology switching) | [![img](https://i1.hdslb.com/bfs/archive/4f60453cb37ce5e4f593f03084dbecd0fdddc27e.jpg)](https://www.bilibili.com/video/BV1UtQfYJExu) |
| Indoor walking (0.5%–1% error) | [![img](https://i1.hdslb.com/bfs/archive/10e501bc7a93c77c1c3f41f163526b630b0afa3f.jpg)](https://www.bilibili.com/video/BV18Q9JYEEdn/) |
| Stair climbing (height error < 5 cm) | [![img](https://i0.hdslb.com/bfs/archive/c469a3dd37522f6b7dcdbdbb2c135be599eefa7b.jpg)](https://www.bilibili.com/video/BV1VV9ZYZEcH/) |
| Outdoor walking (380 m, 3.3% error) | [![img](https://i0.hdslb.com/bfs/archive/481731d2db755bbe087f44aeb3f48db29c159ada.jpg)](https://www.bilibili.com/video/BV1BhRAYDEsV/) |
| Voice interaction + navigation | [![img](https://i2.hdslb.com/bfs/archive/5b95c6eda3b6c9c8e0ba4124c1af9f3da10f39d2.jpg)](https://www.bilibili.com/video/BV1HCQBYUEvk/) |
| Face tracking + laser spot tracking | [![img](https://i0.hdslb.com/bfs/archive/5496e9d0b40915c62b69701fd1e23af7d6ffe7de.jpg)](https://www.bilibili.com/video/BV1faG1z3EFF/) |
| AR glasses head-following | [![img](https://i1.hdslb.com/bfs/archive/9e0462e12bf77dd9bbe8085d0d809f233256fdbd.jpg)](https://www.bilibili.com/video/BV1pXEdzFECW) |
| YOLO drone tracking | [![img](https://i1.hdslb.com/bfs/archive/a5ac45ec76ccb7c3fb18de9c6b8df48e8abe2b54.jpg)](https://www.bilibili.com/video/BV18v8xzJE4G) |
| Gimbal + fixed camera collaboration | [![img](https://i2.hdslb.com/bfs/archive/07ac6082b7efdc2e2d200e18fc8074eec1d9cfba.jpg)](https://www.bilibili.com/video/BV1fTY7z7E5T) |
| Multiple SLAM integrations | [![img](https://i1.hdslb.com/bfs/archive/f299bafc7486f71e061eb31f9f00347063e1e621.jpg)](https://www.bilibili.com/video/BV1ytMizEEdG) |

---

## 📨 Contact

| Email | Affiliation |
| --- | --- |
| [sunminxing20@mails.ucas.ac.cn](mailto:sunminxing20@mails.ucas.ac.cn) | Institute of Optics and Electronics, CAS |

> This repository is under active development. Issues and PRs are welcome.

---

# CAPO-LeggedRobotOdometry 🦾 (中文)

**CAPO-LeggedRobotOdometry** 是一个面向腿式机器人的**纯本体感知里程计库**，核心是**只依赖 IMU 和关节电机数据**的**可移植 C++ 估计器**。

其中，核心算法实现位于 **`FusionEstimator/fusion_estimator.h`**；  
**`fusion_estimator_node.cpp`** 提供该估计器的 **ROS2 封装**；  
**`Matlab/`** 文件夹提供 **MATLAB 与 C++ 混合编译**及离线验证示例。

另外，仓库中的 **`Matlab/Comparison/invariant-ekf/`** 提供了 **`invariant-ekf`** 的 MATLAB 混编方法，便于做离线对比与基线比较。

在 3D 闭环运动实验（水平 **200 m**、竖直 **15 m**）中：璇玑动力公司的点足机器人 A 的水平与竖直误差分别为 **0.1638 m** 和 **0.219 m**；轮足机器人 B 的对应误差分别为 **0.2264 m** 和 **0.199 m**。

---

## 📄 论文

**Contact-Anchored Proprioceptive Odometry for Quadruped Robots**（arXiv:2602.17393）

* 论文链接：https://arxiv.org/abs/2602.17393

如果本仓库对你的研究有帮助，欢迎引用论文。

---

## 📦 数据共享

为便于大家快速验证本仓库的有效性，我们提供了 **Go2-EDU** 的示例数据，包含**实拍录像**、离线 CSV 以及可直接用于本节点的 **ROS bag topic / message**，用于快速复现与 sanity check。

* 数据下载（GitHub Releases）：https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry/releases/tag/DataForTest
* 推荐下载该 release 中的 `GO2Flat`、`GO2Stairs`、`MPXY150Z10`、`MWXY150Z10`、`robot_flat_1_compress.zip`、`robot_stairs_1_compress.zip`

> 说明：该 Go2-EDU 平台的 IMU **yaw 漂移较明显**，因此里程计精度通常会**劣于**论文中 Astrall 机器人 A 与 B 的实验结果。

---

## ✨ 算法特性

| 类别 | 说明 |
| --- | --- |
| **双足 / 四足 / 轮足统一支持** | 运行中自动识别支撑足，支持站立、行走和形态切换。 |
| **仅依赖 IMU 与关节电机数据** | 核心估计器只依赖 IMU 和关节电机测量即可工作，不依赖相机或激光雷达。 |
| **MATLAB / C++ 混编对比** | `Matlab/` 中提供 MATLAB + MEX 调用同一套 C++ 核心的方法，`Matlab/Comparison/invariant-ekf/` 中提供了 `invariant-ekf` 的 MATLAB 混编方法，便于离线比较。 |
| **全 3D + 平面 2D** | 同时发布 6DoF 里程计 `SMX/Odom` 与压平后的 `SMX/Odom_2D`。 |
| **纯 C++ 可移植核心** | `FusionEstimator/` 中是独立的纯 C++ 融合估计实现，便于脱离 ROS2 使用。 |
| **参数可调** | `config.yaml` 中的关键参数和阈值可按平台进行调整。 |

---

## 🏗️ 生态仓库一览

| 范畴 | 仓库 | 功能简介 |
| --- | --- | --- |
| **底层驱动** | https://github.com/ShineMinxing/Ros2Go2Base | DDS 桥、Unitree SDK2 控制、点云→Scan、TF 工具 |
| **里程计** | **CAPO-LeggedRobotOdometry（本仓库）** | 纯本体多传感器融合，发布 `SMX/Odom` / `SMX/Odom_2D` |
| **SLAM / 建图** | https://github.com/ShineMinxing/Ros2SLAM | 集成 Cartographer 3D、KISS-ICP、FAST-LIO2、Point-LIO 等 |
| **语音 / LLM** | https://github.com/ShineMinxing/Ros2Chat | 离线 ASR + OpenAI Chat + 语音合成 |
| **图像处理** | https://github.com/ShineMinxing/Ros2ImageProcess | 相机、光点 / 人脸 / 无人机检测 |
| **吊舱跟随** | https://github.com/ShineMinxing/Ros2AmovG1 | Amov G1 吊舱控制与目标跟踪 |
| **工具集** | https://github.com/ShineMinxing/Ros2Tools | 蓝牙 IMU、手柄映射、吊舱闭环、数据记录等 |

> ⚠️ 按需克隆：如果只做状态估计，本仓库即可独立使用；如果要建图，通常可以配合 `Ros2SLAM` 和 `Ros2Go2Base` 一起使用。

---

## 📂 仓库结构

```text
CAPO-LeggedRobotOdometry/
├── CMakeLists.txt
├── package.xml
├── config.yaml
├── fusion_estimator_node.cpp        # ROS2 节点封装
├── FusionEstimator/                 # 纯 C++ 本体里程计核心
│   ├── Estimators/
│   ├── fusion_estimator.h           # 估计器主入口
│   ├── LowlevelState.h
│   ├── SensorBase.cpp
│   ├── SensorBase.h
│   ├── Sensor_IMU.cpp
│   ├── Sensor_IMU.h
│   ├── Sensor_Legs.cpp
│   ├── Sensor_Legs.h
│   └── Readme.md
├── Matlab/                          # MATLAB + MEX 混编示例
│   ├── build_mex.m
│   ├── fusion_estimator.m
│   ├── fusion_estimator_mex.cpp
│   ├── Comparison/
│   │   └── invariant-ekf/           # invariant-ekf 的 MATLAB 混编与对比示例
│   └── ...                          # 可选测试数据通过 GitHub Releases 提供
├── Plotjuggler.xml
└── Readme.md
```
---

本仓库有意分成三层：

### 1）估计器核心
真正的本体感知里程计算法位于 **`FusionEstimator/fusion_estimator.h`** 及其相关文件中。  
这是本仓库的**主要纯 C++ 估计器核心**，设计目标是仅依赖 **IMU 和关节电机数据**。

### 2）ROS2 封装层
**`fusion_estimator_node.cpp`** 负责把该估计器封装成 ROS2 节点，主要处理：

* Topic 订阅
* 参数读取
* 消息转换
* 里程计发布

### 3）MATLAB 混编与对比
**`Matlab/`** 中提供了 MATLAB + MEX 调用同一套 C++ 核心的方法。  
同时，**`Matlab/Comparison/invariant-ekf/`** 中还提供了 **`invariant-ekf`** 的 MATLAB 混编方法，便于做离线对比与基线评测。

---

## ⚙️ 参数配置（`config.yaml`）

下面列出当前 ROS2 节点最常用的参数。完整注释请查看 `config.yaml`。

| 参数名 | 常见取值 | 说明 |
| --- | ---: | --- |
| `sub_imu_topic` | `SMX/Go2IMU` | IMU 订阅 Topic |
| `sub_joint_topic` | `SMX/Go2Joint` | 关节状态订阅 Topic |
| `sub_mode_topic` | `SMX/SportCmd` | 复位 / 模式 Topic |
| `pub_odom_topic` | `SMX/Odom` | 全 3D 里程计输出 |
| `pub_odom2d_topic` | `SMX/Odom_2D` | 平面里程计输出 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `base_frame` | `base_link` | 机体坐标系 |
| `base_frame_2d` | `base_link_2D` | 平面机体坐标系 |
| `imu_data_enable` | `true` | 是否启用 IMU |
| `leg_pos_enable` | `true` | 是否启用腿部位置运动学 |
| `leg_vel_enable` | `true` | 是否启用腿部速度运动学 |
| `leg_ori_enable` | `false` | 是否启用基于运动学的航向修正 |
| `leg_ori_init_weight` | `0.001` | 姿态修正初始权重 |
| `leg_ori_time_wight` | `1000.0` | 姿态修正随时间增长的权重 |
| `leg_velcke_enable` | `false` | 是否启用 IKVel-CKF / CAPO-CKE |
| `foot_force_threshold` | `20.0` | 触地阈值 |
| `min_stair_height` | `0.08` | 最小台阶高度假设 |

> 说明：
> * 具体参数需要根据机器人平台做调整。
> * `config.yaml` 中有些字段可能用于实验、兼容旧版本，或特定分支。
> * 如果你只移植 `FusionEstimator/` 纯 C++ 核心，一般只需要保留估计器真正需要的参数，不必保留 ROS2 命名相关字段。

---

## ⚙️ 编译与运行

### 克隆仓库

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --depth 1 https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry.git
cd ~/ros2_ws
colcon build --packages-select fusion_estimator
source install/setup.bash
ros2 run fusion_estimator fusion_estimator_node
```

## 📑 节点接口

```text
/fusion_estimator_node (rclcpp)
├─ 发布
│   • SMX/Odom         nav_msgs/Odometry (odom → base_link)
│   • SMX/Odom_2D      nav_msgs/Odometry (odom → base_link_2D)
├─ 订阅
│   • SMX/Go2IMU       sensor_msgs/Imu
│   • SMX/Go2Joint     std_msgs/Float64MultiArray
│       数据布局：
│         data[0..11]   = 12 个关节位置 q
│         data[12..23]  = 12 个关节速度 dq
│         data[24..27]  = 12 个关节力矩 tau / 足端接触力
│   • SMX/SportCmd     std_msgs/Float64MultiArray
│       复位示例：
│         data[0] == 25140000  → 复位估计器
└─ 若需要 TF，一般由其他节点 / 工具单独发布
```

---

## 🧠 算法概览（简述）

1. **触地检测**  
   基于足端力 / 阈值逻辑判定支撑腿。

2. **前向运动学**  
   根据关节测量计算足端在机身坐标系中的位置与速度。

3. **落足点锚定**  
   在触地时记录落足点，在支撑期将接触点作为世界系间歇约束。

4. **高度稳定**  
   利用支撑平面高度逻辑与置信度衰减来降低长时程高度漂移。

5. **可选 IKVel-CKF（CAPO-CKE）**  
   抑制编码器速度尖峰，使足端速度估计更平滑。

6. **可选航向稳定**  
   利用多接触几何一致性来减小原地站立时的 IMU yaw 漂移。

7. **2D 投影**  
   压平 roll / pitch，保留 yaw 与平面运动，发布 `SMX/Odom_2D`。

完整推导请参考论文。

---

## 🔧 在 ROS2 之外复用 `FusionEstimator/`

如果你**不需要 ROS2**，真正可复用的主体主要就是 `FusionEstimator/`。

一个典型迁移流程是：

1. 保留 `FusionEstimator/` 下的纯 C++ 核心
2. 用你自己的传感器接口替换 ROS2 订阅
3. 准备 IMU、关节位置 / 速度、触地 / 足端力输入
4. 在你的主循环中调用估计器
5. 将输出接到你的中间件 / 控制系统

这对以下场景很有帮助：

* ROS1 迁移
* 嵌入式部署
* 离线数据回放
* MATLAB / 仿真验证
* 自定义机器人软件框架

---

## 🧪 MATLAB / MEX（可选）

`Matlab/` 文件夹提供了将本仓库 C++ 估计器核心编译成 MATLAB 可调用 MEX 模块的示例。

典型流程：

```matlab
cd Matlab
build_mex
fusion_estimator
```

该文件夹中的主要文件：

* `build_mex.m`：MEX 编译脚本
* `fusion_estimator_mex.cpp`：MEX 桥接文件，负责封装 C++ 估计器核心
* `fusion_estimator.m`：MATLAB 侧调用示例
* 离线测试用示例数据与压缩包：统一通过 GitHub Releases 提供：https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry/releases/tag/DataForTest

这部分特别适合以下需求：

* 在 MATLAB 中验证 C++ 融合器行为
* 快速做离线结果对比
* 不依赖 ROS2 复用同一套 `FusionEstimator/`
* 在把修改重新合回 ROS2 节点之前，先做算法调试

---

## 📈 可视化

仓库中附带了 `Plotjuggler.xml`，可用于 PlotJuggler 的快速可视化与调试。

你可以用它观察：

* 里程计输出
* IMU 相关信号
* 关节 / 足端力相关量
* 站立、行走、复位过程中的估计器行为

---

## 🎥 视频演示

| 主题 | 链接 |
| --- | --- |
| 纯里程计建图（形态切换） | [![img](https://i1.hdslb.com/bfs/archive/4f60453cb37ce5e4f593f03084dbecd0fdddc27e.jpg)](https://www.bilibili.com/video/BV1UtQfYJExu) |
| 室内行走（误差 0.5%–1%） | [![img](https://i1.hdslb.com/bfs/archive/10e501bc7a93c77c1c3f41f163526b630b0afa3f.jpg)](https://www.bilibili.com/video/BV18Q9JYEEdn/) |
| 爬楼梯（高度误差 < 5 cm） | [![img](https://i0.hdslb.com/bfs/archive/c469a3dd37522f6b7dcdbdbb2c135be599eefa7b.jpg)](https://www.bilibili.com/video/BV1VV9ZYZEcH/) |
| 户外行走（380 m，误差 3.3%） | [![img](https://i0.hdslb.com/bfs/archive/481731d2db755bbe087f44aeb3f48db29c159ada.jpg)](https://www.bilibili.com/video/BV1BhRAYDEsV/) |
| 语音交互 + 地图导航 | [![img](https://i2.hdslb.com/bfs/archive/5b95c6eda3b6c9c8e0ba4124c1af9f3da10f39d2.jpg)](https://www.bilibili.com/video/BV1HCQBYUEvk/) |
| 人脸识别跟踪 + 光点跟踪 | [![img](https://i0.hdslb.com/bfs/archive/5496e9d0b40915c62b69701fd1e23af7d6ffe7de.jpg)](https://www.bilibili.com/video/BV1faG1z3EFF/) |
| AR 眼镜头部运动跟随 | [![img](https://i1.hdslb.com/bfs/archive/9e0462e12bf77dd9bbe8085d0d809f233256fdbd.jpg)](https://www.bilibili.com/video/BV1pXEdzFECW) |
| YOLO 无人机识别与跟随 | [![img](https://i1.hdslb.com/bfs/archive/a5ac45ec76ccb7c3fb18de9c6b8df48e8abe2b54.jpg)](https://www.bilibili.com/video/BV18v8xzJE4G) |
| 吊舱 + 固定相机协同 | [![img](https://i2.hdslb.com/bfs/archive/07ac6082b7efdc2e2d200e18fc8074eec1d9cfba.jpg)](https://www.bilibili.com/video/BV1fTY7z7E5T) |
| 多种 SLAM 方法集成 | [![img](https://i1.hdslb.com/bfs/archive/f299bafc7486f71e061eb31f9f00347063e1e621.jpg)](https://www.bilibili.com/video/BV1ytMizEEdG) |


---

## 📨 联系方式

| 邮箱 | 单位 |
| --- | --- |
| [sunminxing20@mails.ucas.ac.cn](mailto:sunminxing20@mails.ucas.ac.cn) | 中国科学院光电技术研究所 |

> 本仓库持续开发中，欢迎 Issue / PR 交流与贡献。
