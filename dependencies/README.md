# 定位建图依赖

基线：Ubuntu 24.04、ROS 2 Jazzy、C++17。依赖独立放在 `vendor/sources`，构建到 `build/mapping`、`install/mapping`，不混入基础工程的 `src` 或 `install/ros`。

## 固定版本

完整提交号见 [navigation.repos](navigation.repos)。源码缓存保留各上游仓库和许可证，不直接复制进本仓库。

| 组件 | 上游 | 选择 |
| --- | --- | --- |
| FAST-LIO2 ROS 2 实现 | [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) | `2fffc570a25d`，包含固定版本的 ikd-Tree 子模块 |
| MID-360 驱动 | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | `21445540f0d1` |
| Livox SDK2 | [Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | `c0796f04c143`，安装到项目内 `install/livox_sdk2` |
| RTAB-Map 核心 | [introlab/rtabmap](https://github.com/introlab/rtabmap) | `faf0ac54d2f5`，`0.23.13-jazzy` |
| RTAB-Map ROS | [introlab/rtabmap_ros](https://github.com/introlab/rtabmap_ros) | `cb2117f1ed66`，`0.23.13` |

当前 RTAB-Map 使用官方 Jazzy **0.23.13 二进制包**，核心和 ROS 源码下载用于审查与后续维护，不重复从源码构建另一套 RTAB-Map。安装脚本解析该上游版本对应的可用 Debian 修订号；如果软件源不再提供该版本就报错，不能静默升级。完整系统依赖仍不是可离线重建的二进制锁；发行版基础镜像和传递依赖可能更新。

HKU 官方仓库也有大写 `ROS2` 分支；本项目当前选择已有 MID-360/driver2 接入的 Ericsii 实现。选择与补丁必须在真实数据回归后再评估，不能仅以能编译作为算法选型的最终证据。

## 下载和构建

在已安装 Jazzy 的开发机交互终端执行：

```bash
bash scripts/install_mapping_dependencies.sh
bash scripts/fetch_navigation_sources.sh
bash scripts/build_mapping_dependencies.sh
```

第一条需要 sudo；后两条在项目内下载和构建，不安装系统级 Livox SDK，也不启动传感器或机器人。下载使用精确提交和递归子模块，保留 `vendor/actual.repos`。构建检查源码版本、应用受版本控制的最小补丁，并独立编译 Livox 驱动和 FAST-LIO2。不要运行上游驱动的 `build.sh`，它会清理相邻工作空间的构建和安装目录。

构建成功的版本、架构和哈希证据位于 `log/mapping/build-summary.txt`，包含各仓库相对 HEAD 的 tracked diff 哈希；保留本地修改，不把固定 HEAD 当作工作区未修改的证明。`vendor` 和构建产物不包含在普通源码快照中；Thor 首次接入需要准备对应依赖，并原生构建，不能复制本机 x86_64 的库到 ARM64 运行。

无传感器输入的进程启动检查：

```bash
source /opt/ros/jazzy/setup.bash
source install/mapping/setup.bash
python3 tests/ros/test_mapping_binaries.py
```

检查实际 FAST-LIO/RTAB-Map 进程、DDS 订阅类型和生效参数，并检查正常退出。只使用独立 localhost 域，不启动驱动、运动节点或传感器数据发布器；日志保留在 `artifacts/mapping-smoke`。其中 identity 外参仅满足空输入启动条件，不是 H2 标定。执行环境必须允许本机 DDS 套接字。

FAST-LIO 补丁处理 C++ 标准选项冲突、先填写当前帧 pose covariance 再发布里程计，以及关闭调试日志时仍写源码目录的问题；不改变估计器或补造速度/协方差。补丁路径：[fast_lio_jazzy.patch](patches/fast_lio_jazzy.patch)。本轮启动检查显式关闭调试日志和 PCD 保存；上游示例配置不直接作为 H2 配置使用。启用日志/地图保存的路径仍需后续封装，不能直接作为产品持久化方案。

## 接入边界

- MID-360 使用 `livox_ros_driver2/msg/CustomMsg`（驱动 `xfer_format=1`、LIO `lidar_type: 1`）与雷达内置 IMU。`header.stamp` 与 `timebase` 使用同一纳秒基准，逐点时刻为基准加 `offset_time(ns)`；保留 line/tag。不能把当前移植中按旋转扫描角度合成时间的通用 PointCloud2 路径用于 MID-360。
- 锁定驱动在无同步模式下使用主机包到达时间；存在逐点时间不代表已经验证硬同步。PTP/GPS、网络抖动、时间单调性、扫描跨度和 IMU 覆盖须用真实数据检查。FAST-LIO 的简易软件对齐保持关闭，不能代替时间标定。
- 锁定版本驱动原始加速度单位为 g、角速度为 rad/s；其 IMU frame 还需核对。FAST-LIO 内部按静止加速度模长归一化。后续项目 IMU 适配须显式形成 SI 单位、正确 frame 和 orientation 有效性契约，原始 IMU 不直接作为 RTAB-Map 的重力姿态。
- 上游 FAST-LIO 的 `camera_init` / `body` 是其估计世界与 IMU 坐标系，不是已经验证的项目 `odom` / H2 `base_link`。还需重力、地面参考、安装外参和必要关节链适配；不能仅改 frame 名字。
- RTAB-Map 后续接收已去畸变点云和经过适配的完整局部里程计，关闭其 ICP 里程计和直接 TF 校正发布。纯激光后端、重定位检索与审核封装仍按总体架构接入。
- 下载、安装、编译和无输入启动验证分别记录。没有真实 MID-360 bag 回归、Thor 构建或 H2 验收时，不表示导航功能已完成。

当前证据见 [实现状态](../docs/implementation_status.md)。基础工程检查仍使用 `tools/h2ctl.py check --profile core/ros`；算法依赖构建不会自动进入运动链。
