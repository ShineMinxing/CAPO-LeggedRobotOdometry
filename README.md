# CAPO-LeggedRobotOdometry 🦾  [![License](https://img.shields.io/badge/License-See%20LICENSE-blue.svg)](LICENSE)

**CAPO-LeggedRobotOdometry** is a **pure proprioceptive odometry library** for legged robots, implemented with a **portable C++ estimator core** that depends only on **IMU and joint motor data**.

The core estimation logic is implemented in **`FusionEstimator/fusion_estimator.h`**. The file **`fusion_estimator_node.cpp`** provides a **ROS 2 wrapper** around this estimator, while the **`Matlab/`** folder contains examples for **MATLAB + C++ mixed compilation** and offline evaluation.

For side-by-side comparison, **`Matlab/Comparison/invariant-ekf/`** provides a MATLAB mixed-compilation workflow for **`invariant-ekf`**, making it easier to compare this repository against a representative open-source legged odometry baseline.

In 3D closed-loop trials (a **200 m** horizontal and **15 m** vertical loop), Astrall point-foot robot A achieves **0.1638 m** horizontal error and **0.219 m** vertical error; for wheel-legged robot B, the corresponding errors are **0.2264 m** and **0.199 m**.

Welcome to add my wechat 401435318 and join a wechat group.

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