# MID-360S 静态 FAST-LIO2 回放

在开发机使用已回传的 bag，检查 FAST-LIO2 初始化与静止轨迹。当前实验使用固定的单位旋转和零平移作为雷达—IMU 外参假设，启动时需要显式设置实验参数。外参标定与动态精度验收另行进行。

## 已定位的地图参数问题

锁定版本 FAST-LIO 的局部地图在距边界 `1.5 × mapping.det_range` 以内时触发移动。地图边长必须满足 `cube_side_length > 3 × mapping.det_range`，使静止初始位置处在移动阈值之外。

首次本地回放把地图边长设成 200 m、检测距离设成 100 m，地图半宽只有 100 m，却要求距边界超过 150 m。算法因此在静止时反复移动地图盒子并删除有效点。内部日志显示，早期部分帧删除约 600 个地图点，树内仅剩约 60 点，随后匹配和滤波状态失稳。

在同一台开发机上，使用同一二进制、同一数据、同一播放速度，仅把边长改为 1000 m，得到以下对照。

| 项目 | 边长 200 m | 边长 1000 m |
| --- | --- | --- |
| 完整输入观察计数 | 300 点云、6002 IMU | 300 点云、6002 IMU |
| 里程计输出 | 297 条 | 297 条 |
| 输出时间跨度 | 29.601 s | 29.601 s |
| 首末位置差 | 700.419 m | 0.02035 m |
| 相对首个输出的最大位置变化 | 700.419 m | 0.03221 m |
| 首末姿态差 | 52.483° | 0.509° |

配置使用 `cube_side_length: 1000.0` 与 `mapping.det_range: 100.0`。启动入口会在加载参数时拒绝不满足边界条件的组合。这些数字描述本次静态数据上的变化，实验外参、时间同步及 H2 机体坐标仍需核验。

## 在开发机重现

在仓库根目录加载已构建的环境。首次拉取新入口后，先刷新项目包。

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/h2ctl.py check --profile ros --jobs 2
source install/mapping/local_setup.bash
source install/ros/local_setup.bash
export ROS_DOMAIN_ID=167
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
ros2 launch h2_bringup lio_static_replay.launch.py \
  allow_experimental_extrinsics:=true
```

保持此终端运行。在第二个开发机终端进入同一仓库，加载相同环境后播放原始 bag。

```bash
source /opt/ros/jazzy/setup.bash
source install/mapping/local_setup.bash
source install/ros/local_setup.bash
export ROS_DOMAIN_ID=167
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
ros2 bag play "$PWD/data/from-h2/first-test-001/livox-stationary-retry-01/bag" \
  --rate 0.5 \
  --delay 2 \
  --read-ahead-queue-size 10000 \
  --wait-for-all-acked 1000 \
  --disable-keyboard-controls \
  --remap /livox/lidar:=/h2_lio_replay/raw_lidar \
          /livox/imu:=/h2_lio_replay/raw_imu
```

半速播放 30 秒数据大约需要一分钟，结束后再等几秒让队列处理完成。随后在第一个终端按 Ctrl+C。程序日志应出现 `IMU Initial Done` 和 `Initialize the map kdtree`。

主要输出为 `/h2_lio_replay/odometry`、`/h2_lio_replay/world_cloud` 和 `/h2_lio_replay/body_cloud`。点云帧会依次用于起点、IMU 初始化和初始地图建立，因此本次 300 帧输入对应约 297 条里程计输出。输出保留 FAST-LIO 原生 `camera_init/body` 坐标，后续需完成机体适配。

本入口加载固定实验配置并隔离 ROS 话题，默认关闭源码目录调试日志和 PCD 保存。上述命令用于复现节点与输入链路，统计报告和轨迹保存见本地分析结果。

## 本次数据的本地结果

开发机回传目录 `data/from-h2/first-test-001/analysis/fastlio-static-003/` 保存正确参数回放的 `report.json`、`parameters.yaml`、`odometry.csv`、节点日志、末帧点云与内部状态日志。`fastlio-static-002/` 保留错误参数的对照。通过新增项目入口的完整复测记录在 `analysis/fastlio-static-004/`，消息计数和位置变化与修正参数后的直接回放相同。修复前后的轨迹图位于 `analysis/static_replay_comparison.png`。
