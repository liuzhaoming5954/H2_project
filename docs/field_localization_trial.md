# 现场工程师：用一段新的静止扫描试定位

本轮回答一个问题：**让雷达停在已知地点附近，能否在之前的点云地图中找回它的位置和方向？** 推荐用 13:31 的第二张地图。先录 10 秒，再在 PC3 或已装好本工程的开发机运行匹配。程序只读取文件，不发布 TF、不发机器人指令。它需要大致知道雷达在哪、朝哪；不能从未知房间自动找位置，也不提供行走中的持续定位。

## 1. 拿到代码和地图

**已在 PC3 完成过采集的工程师**：停止旧采集，在原项目目录执行。`git status --short` 如果有你自己的未提交改动，先保存再更新，不要重置现场目录。

```bash
cd ~/H2_project
git status --short
git fetch origin
git switch codex/h2-readonly-first-test
git pull --ff-only origin codex/h2-readonly-first-test
git rev-parse --short HEAD
(cd maps/site-001 && sha256sum -c SHA256SUMS)
```

应看到三张 `map.ply` 和六张 PNG 都显示 `OK`。有任何 `FAILED` 就先不要试定位。如果是**新电脑，还没有工程目录**，先用下面的命令克隆；首次搭建采集环境按[首次上机教程](first_hardware_test_tutorial.md)完成，再从第 2 步继续。

```bash
cd ~
git clone -b codex/h2-readonly-first-test https://github.com/liuzhaoming5954/H2_project.git
cd H2_project
(cd maps/site-001 && sha256sum -c SHA256SUMS)
```

本次匹配工具是 Python 脚本；以前已能采集的 PC3 不用为它重新编译 FAST-LIO 或启动 RTAB-Map。两台机器各自在运行工具前加载本机 ROS 环境。下面以项目在 `~/H2_project` 为例。

## 2. 复现一个能说清楚的位置

1. 请采集人员指出 **13:31 这段**开始推椅子的地面位置、雷达的朝向，以及安装方向。可参照[轨迹图](../maps/site-001/rec_20261118_133127/trajectory.png)和[三维点云投影](../maps/site-001/rec_20261118_133127/pointcloud.png)。第一次最容易复现的是原来的椅子、相近高度和相同安装朝向。窗外远景或单一大平面不适合首试，扫描中最好能看到固定墙角、门框等多个方向的结构。
2. 如果已改为 **H2 头顶安装**，保持静止，固定头颈姿态；初值要描述**新雷达**相对原建图起点的位置和三轴姿态。不能用脚下位置或机器人正面代替雷达。三张地图的 Z 都不是可直接当作向上的楼层高度轴：第二段开头静止 IMU 给出的近似向上方向为地图坐标 `(0.063, 0.025, -0.998)`。若仅把雷达垂直抬高 1 m、安装方向完全不变，平移初值约增加这个向量；如果安装翻转/旋转，还要改姿态。若无法确定安装方向，先用原椅子复现一次，再测 H2 的新安装方向。
3. 在地上标记雷达投影点和朝向，记下雷达高度、安装方式及现场参照物。整个 10 秒保持静止。程序要求初值在正确位置**附近**；可先争取估到 0.5 m、10° 以内。这是操作建议，不是保证可找回的范围。

## 3. 在 PC3 新录 10 秒（已有新静止包可跳到第 4 步）

开两个终端 A、B。在**两个终端分别**执行：

```bash
cd ~/H2_project
source /opt/ros/jazzy/setup.bash
source install/mapping/local_setup.bash
source install/ros/local_setup.bash
export ROS_DOMAIN_ID=81 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=''
export ROS_LOG_DIR="$PWD/log/readonly"
mkdir -p "$ROS_LOG_DIR" "$PWD/data/site-001"
```

只在 **A** 启动已经验证过的只读雷达驱动。如果已有同一个驱动在运行，复用它，不要重复启动。

```bash
ros2 launch h2_bringup system.launch.py mode:=hardware_readonly \
  lidar_config:="$PWD/config/local/mid360s.json"
```

只在 **B** 运行下列命令。保持雷达不动，等程序自己结束。本次 `capture` 目录必须由录制程序新建。

```bash
trial="$PWD/data/site-001/localize-$(date +%Y%m%d-%H%M%S)"
mkdir "$trial"
python3 tools/readonly_capture.py record --duration 10 \
  --output "$trial/capture" --config "$PWD/config/local/mid360s.json" \
  --max-size-mib 256 --min-free-mib 2048
ls "$trial/capture/bag/metadata.yaml"
```

录制检查通过再继续；若失败请保留目录，参照[采集教程的故障排查](field_pointcloud_capture.md)。完成录制后即可在 A 按 `Ctrl+C` 关闭本次启动的驱动。下步不需要机器人或雷达在线。

## 4. 计算位置

继续在 B 操作。若换到开发机，把本次 `localize-...` 整个目录复制到开发机，按第 3 步加载环境，再把 `trial` 设置为复制过来的目录的绝对路径。检查依赖：

```bash
/usr/bin/python3 -c 'import cv2, numpy, yaml, PIL, rosbag2_py; from livox_ros_driver2.msg import CustomMsg; print("环境就绪")'
```

若缺 `cv2`，用 `sudo apt install python3-opencv`；若缺 `livox_ros_driver2`，先核对是否加载 `install/mapping/local_setup.bash`。不要在缺依赖的终端反复重录。

下面的六个零**只适用于雷达基本回到第二段建图起点、高度和安装朝向都相近**。其他情况先替换 `--seed` 后的六个数，顺序为地图坐标 `x y z`（米），然后 `roll pitch yaw`（度；绕 X、Y、Z，组合为 `Rz·Ry·Rx`）。

```bash
result="$trial/result-$(date +%Y%m%d-%H%M%S)"
mkdir "$result"
/usr/bin/python3 tools/offline_localize.py \
  --map maps/site-001/rec_20261118_133127/map.ply \
  --bag "$trial/capture/bag" --start-s 2 \
  --seed 0 0 0 0 0 0 \
  --report "$result/localization.json" --overlay "$result/overlay.png"
```

`--bag` 指向**直接含 `metadata.yaml` 和所有 `.db3` 分片**的目录。工具检查录制的雷达与 IMU 话题元数据，但匹配只用激光：从第 2 秒附近取三帧，间隔约半秒，分别求位姿。要测试另一段地图，既要换 `--map`，也要回到对应现场、换为它自己的初值。每次试验新建结果目录，已有结果不会覆盖。

## 5. 判读与回传

| 终端 / JSON | 含义和下一步 |
| --- | --- |
| `"status": "candidate"`，退出码 0 | 三帧在初值附近的几何匹配一致。`xyz_m` 是中间帧的雷达位置；`T_map_lidar` 将雷达坐标变换为该张地图坐标。打开同目录 `overlay.png`：橙色是新扫描，灰蓝色是老地图。检查三个投影里的墙角、门框是否同时对齐。 |
| `"status": "rejected"`，退出码 2 | 此次不出位姿。`insufficient_nearby_map_points` 先查地图、坐标和初值；`insufficient_3d_geometry` 试有墙角/门框的位置；`overlap_residual_or_seed_gate` 查安装、环境遮挡、朝向；`stationary_poses_disagree` 检查是否挪动或转头。保留 JSON 和原包，不要调松阈值硬凑结果。 |
| `error` 或命令报错，退出码非零 | 查输出目录、依赖、包是否录完、话题和雷达帧名。当前支持本项目的 ROS 2 SQLite/CDR、`/livox/lidar`、`livox_frame`。如果文件无法打开，可能没有生成报告。 |

工具将每帧约四分之一的抽样点留出，不参加拟合：这些点中至少 65% 要在地图点的 20 cm 内，中位距离不超过 12 cm。三帧候选之间最大位置差不超过 12 cm，最大姿态差不超过 2°。这些是筛选门槛；重复走廊或相似房间仍可能匹配到错处。**一次 `candidate` 不代表 H2 通过定位验收。**

建议先在地面标记 A 原地独立录两次，再人工把雷达移到可辨认的标记 B、停稳后另录一次，使用 B 的大致初值。用卷尺实测 A、B 的雷达位置间距，与两份 `xyz_m` 的距离对照。记录 H2 的头姿及雷达高度，结果只描述雷达，不代表 `base_link`。

请回传每次完整的 `localize-.../` 目录（包含 `capture/bag/`、所有分片、`result-.../localization.json` 和候选成功时的 `overlay.png`），附上地图名、`git rev-parse HEAD`、初值、实际标记/朝向和实测间距。可以复制到开发电脑 `data/from-h2/site-001/` 下。原始 bag、实机配置和现场记录留在团队内部，不用推回公开仓库。
