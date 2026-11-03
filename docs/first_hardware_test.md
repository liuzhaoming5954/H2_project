# 第一轮 H2 只读上机测试

本轮目标是确认 PC3/Thor 的构建兼容性、MID-360S 原始数据和 H2 状态订阅，保存可带回开发机分析的数据。在 PC3 上填写实际雷达与接收网卡地址。

本轮不启动自主行走、Nav2、运动许可、H2 指令 writer 或估计器 TF。雷达采集入口只启动 Livox 驱动；独立状态探针只订阅 H2。这里的“只读”针对 H2 运动控制，Livox SDK 启动时会配置雷达输出地址和数据格式。

## 1. 代码与环境

在 PC3 的终端执行下面的首次克隆命令。仓库已公开，可通过 HTTPS 匿名克隆。首次下载代码和依赖需要 PC3 能访问 GitHub 和 Ubuntu/ROS 软件源。

```bash
git clone --branch codex/h2-readonly-first-test --single-branch \
  https://github.com/liuzhaoming5954/H2_project.git H2_project
cd H2_project
git rev-parse HEAD
test -f src/h2_bringup/config/mid360s.example.json
```

记录输出的提交 SHA，便于复现实机结果。部署版本须包含 `src/h2_bringup/config/mid360s.example.json` 和对应的启动校验。此前的 `05eb678` 只接受普通 MID-360 配置。已有工作区时，先用 `git status --short` 检查本地改动。确认工作区干净后，可执行 `git fetch origin refs/heads/codex/h2-readonly-first-test` 和 `git switch --detach FETCH_HEAD` 切到刚下载的版本，再记录 SHA 和执行模板检查。更新后执行下方的构建检查，以刷新已安装的启动文件。若模板检查失败，先核对分支与提交。

后续命令均在仓库根目录执行。普通源码快照不包含 `vendor` 和编译产物，PC3 需要准备固定依赖并进行 ARM64 原生构建。保留原厂 BSP、JetPack 和服务；先清点环境，再安装缺少的依赖。x86_64/CI 通过不代表 Thor/ARM64 已经验证，本轮正在验证这一项。

```bash
mkdir -p data/first-test-001 log/readonly config/local
python3 tools/readonly_environment.py --report data/first-test-001/environment-before.json
ip -br address
ip route
```

环境报告包含架构、内核、NVIDIA 包、工作区剩余磁盘、ROS 工具和网卡地址。生成报告不代表验收通过；确认 `target_arm64=true`，并检查 `ros_prerequisites`。如果此终端尚未 source Jazzy，先保留原始清单，加载环境后再运行 doctor。

现场记录 PC3 雷达网卡 IP、用于回传数据的 SSH 地址和用户名，以及可选的 H2 内部 DDS 网卡名。雷达网卡和 H2 状态网卡可能不同，不能从示例地址推定。本轮工具只读取网卡清单，不自动更改网络。

缺少 Jazzy 时使用 `bash scripts/install_ros_jazzy.sh`；确认缺少定位依赖后使用 `bash scripts/install_mapping_dependencies.sh`。安装需要终端交互 sudo，脚本不接收密码。随后执行：

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/h2ctl.py doctor --profile ros
bash scripts/fetch_navigation_sources.sh
bash scripts/build_mapping_dependencies.sh --jobs 2
python3 tools/h2ctl.py check --profile ros --jobs 2
```

`check` 中的运动测试使用独立 ROS 域和 mock，仍是模拟接口检查，不代表 H2 运动接口验证。依赖构建证据在 `log/mapping/build-summary.txt`。首次上机须确认 `architecture=aarch64`，不能复制开发机的 x86_64 库到 Thor 使用。

## 2. 填写 MID-360S 配置

```bash
cp -n src/h2_bringup/config/mid360s.example.json config/local/mid360s.json
```

在 PC3 输入雷达标签或现场记录里的固定 IPv4，然后查看 PC3 的出接口和源地址：

```bash
read -r -p 'MID-360S 固定 IPv4: ' h2_lidar_ipv4
ip route get "$h2_lidar_ipv4"
```

确认 `dev` 是接雷达的网卡，`src` 是该网卡的实际 IPv4。若选到了默认路由或其他网卡，先核对 PC3 雷达网口的地址、掩码和物理连接。用编辑器打开 `config/local/mid360s.json`，填写以下字段：

| JSON 字段 | 填写值 |
| --- | --- |
| `lidar_configs[0].ip` | MID-360S 固定 IPv4 |
| `Mid360s.host_net_info[0].host_ip` | PC3 雷达网卡 IPv4，即已确认的 `src` |

保持 `Mid360s` 的大小写、`host_net_info` 的单元素数组结构、模板端口、`lidar_type: 8` 和零 SDK 外参。雷达和 PC3 地址必须不同。可在启动前做一次离线校验：

```bash
PYTHONPATH="$PWD/src/h2_bringup/launch" python3 -c \
  'import sys; from mid360_config import load_mid360_config; print(load_mid360_config(sys.argv[1]))' \
  "$PWD/config/local/mid360s.json"
```

本地配置位于忽略目录 `config/local`，原始数据位于忽略目录 `data`，不直接提交到团队源码仓库。

## 3. 启动雷达采集

在 PC3 的 ROS 终端运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/mapping/local_setup.bash
source install/ros/local_setup.bash
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
export ROS_LOG_DIR="$PWD/log/readonly"
ros2 launch h2_bringup system.launch.py \
  mode:=hardware_readonly \
  lidar_config:="$PWD/config/local/mid360s.json"
```

入口只加载 `livox_ros_driver2_node`，不加载 mock、supervisor、motion_guard、Unitree 控制客户端或 TF 发布器。原始话题为 `/livox/lidar`（CustomMsg）和 `/livox/imu`（Imu），frame 保留 `livox_frame`。按 Ctrl-C 结束采集驱动。

默认 ROS 图限制在 PC3 本机。其他终端的环境不会被 launch 内部设置反向修改，必须在每个检查/录制终端设置相同的 domain 和 LOCALHOST。H2 状态探针是另一套独立 DDS 进程，使用其原厂 domain 0，不与此 ROS 域混用。

## 4. 检查真实数据并录制

另开 PC3 ROS 终端，进入同一仓库根目录，重复上一节 source 和四个 export 后执行：

```bash
python3 tools/readonly_capture.py inspect \
  --duration 10 \
  --report "$PWD/data/first-test-001/livox-inspect.json"

python3 tools/readonly_capture.py record \
  --duration 30 \
  --output "$PWD/data/first-test-001/livox-stationary" \
  --config "$PWD/config/local/mid360s.json" \
  --max-size-mib 512 \
  --min-free-mib 2048
```

先采集静止段。若后续需要运动轨迹，由现场人员通过原厂控制方式操作，本项目继续只记录。每次使用新的输出目录/报告文件；工具拒绝覆盖已有结果。

`inspect` 检查两路实际样本、接收频率、连续性、时间单调、点数、逐点时间跨度及 IMU 数值。`record` 先等待两路数据，再启动明确话题清单的 rosbag2，并确认本次唯一录制器节点的两路类型订阅就绪，才开始录制窗口计时；原始流和录制器就绪分别受 `startup-timeout` 限制。bag 可能包含短暂的订阅预热数据，窗口诊断从就绪后重新统计。结束后检查 bag 元数据是否确有两路样本，保存 `bag/`、`report.json`、`recorder.log`、配置及版本快照。无样本、缺一路、数据不合规或录制异常均返回非零。SDK 初始化失败时驱动可能继续存活，所以不能只看节点/进程存在。

默认最低频率是数据存在检查门槛；需要结合报告中的实际频率和设备配置核对，不将门槛通过当作吞吐性能验收。中途接收间断超过 `max_silence` 也会失败，结束前恢复不能掩盖该间断。

磁盘/目录大小通过轮询监控，是软阈值，录制停止及元数据落盘会有延迟。返回失败时仍保留已采集文件和诊断，不能将残留目录当作完整成功数据。

本锁定驱动的原始加速度为 g、角速度为 rad/s。数据检查不把原始 IMU 当作带有效姿态的标准 SI 接口。CustomMsg 的逐点时间存在，也不证明 PTP/IMU 对齐已成立；报告保留同步与 TF 为未验证，供回放时进一步分析。

## 5. 可选：H2 被动状态采集

构建单独的原生工具：

```bash
bash scripts/fetch_h2_state_source.sh
bash scripts/build_h2_state_probe.sh
```

它锁定官方 SDK 提交，使用 SDK 自带 DDS 独立链接，不构建或运行上游控制示例。SDK 自带的预编译库仍需在 Thor 实际验证 ABI。构建记录在 `log/h2-state/build-summary.txt`。

进入仓库，填写 H2 状态网卡后，用下面的一次性干净环境启动。即使新终端被 `.bashrc` 自动加载 ROS，这种方式也会隔离继承的 ROS、动态库和 DDS 配置，不修改终端启动文件：

```bash
env -i HOME="$HOME" USER="${USER:-$(id -un)}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  "$PWD/install/h2_state_probe/bin/h2_state_probe" \
  --interface REPLACE_WITH_H2_STATE_INTERFACE \
  --output "$PWD/data/first-test-001/h2-state.jsonl" \
  --duration 30 \
  --timeout 5 \
  --sample-hz 50
```

默认订阅 `rt/lowstate` 与 `rt/secondary_imu`，分别保存机身与躯干 IMU 等原始字段。若现场先只核对 lowstate，可显式加 `--lowstate-only`，其成功范围不包含 secondary_imu。程序不实例化 LocoClient、不发布 lowcmd、不申请或释放控制权。

JSONL 保留主机接收时刻、原始 tick/模式、IMU、35 槽 motor_state、遥控字节和 CRC 诊断。官方 H2 示例使用前 31 个关节；其余槽位不推定为有效关节。50 Hz 是最新样本采样，不是无损全速记录；时间基准、单位、姿态顺序和 frame 仍需现场确认，不能直接当作融合后的机器人里程计。

两路状态缺失/过期、采样写入记录中的 CRC 或数值错误、输出上限等会返回非零。CRC/数值检查仅覆盖实际抽样写入的记录，不覆盖抽样间被替换的全部来流；`passed` 的范围也据此限定。输出是采集文件，不是 ROS 运动状态或 TF。

## 6. 核对 PC3 上的测试产物

以下路径以 PC3 上的仓库根目录为起点，使用本说明中的测试编号 `first-test-001`。按步骤完成后，采集数据和相关日志分布如下。

```text
H2_project/
├── data/first-test-001/
│   ├── environment-before.json          # PC3 环境和网络清单
│   ├── livox-inspect.json               # 10 秒点云和 IMU 检查报告
│   ├── livox-stationary/
│   │   ├── bag/
│   │   │   ├── metadata.yaml            # 话题、消息数和录制时长
│   │   │   └── bag_0.db3                # 点云和 IMU 的 SQLite 数据
│   │   ├── report.json                 # 录制结果、诊断与版本信息
│   │   ├── recorder.log                # rosbag2 录制日志
│   │   ├── mid360-config.json           # 本次配置快照，MID-360S 沿用此文件名
│   │   ├── navigation.repos            # 依赖版本快照
│   │   └── build-summary.txt           # Mapping 构建记录副本，源文件存在时复制
│   └── h2-state.jsonl                   # 可选，运行 H2 被动状态探针后生成
├── log/
│   ├── readonly/                       # ROS 启动及运行日志，文件名随启动变化
│   ├── mapping/build-summary.txt        # Mapping 依赖构建记录
│   └── h2-state/build-summary.txt       # 可选，状态探针构建记录
└── config/local/mid360s.json             # 现场填写的采集输入
```

录制命令退出后，查看 `livox-stationary/report.json`，确认 `ok` 为 `true`。再核对 `bag/metadata.yaml` 中的 `/livox/lidar` 和 `/livox/imu`，两路消息数都应大于零。若录制器启动前就缺少一路，可能只有配置快照和失败报告。若录制中断，仍可能留下不完整的 bag。失败时也保留已生成的文件，供回传排查。

回传时复制完整的 `data/first-test-001/` 和第 7 节列出的日志。安装照片、尺寸、探针运行参数和退出码需要现场另行记录。可选的人工操作轨迹应与静止数据分组保存。

第一轮通过的含义是：目标机能够构建、接收并保存这些原始数据，已检查的数据流满足采集工具的结构和连续性条件。LIO 精度、重力/机体 TF、地图定位、导航和 H2 运动停车均有各自后续验收，不由本轮数据接入结果代替。

## 7. 将数据带回开发机

在**开发机**仓库根目录执行，将 `PC3_USER`、`PC3_IP` 和 `/ABSOLUTE_PATH/H2_project` 替换为现场确认的值：

```bash
mkdir -p data/from-h2/first-test-001/logs
rsync -av \
  PC3_USER@PC3_IP:/ABSOLUTE_PATH/H2_project/data/first-test-001/ \
  data/from-h2/first-test-001/
rsync -av \
  PC3_USER@PC3_IP:/ABSOLUTE_PATH/H2_project/log/readonly/ \
  data/from-h2/first-test-001/logs/readonly/
rsync -av \
  PC3_USER@PC3_IP:/ABSOLUTE_PATH/H2_project/log/mapping/build-summary.txt \
  data/from-h2/first-test-001/logs/mapping-build-summary.txt
```

如果执行了 H2 状态探针构建，再复制对应记录：

```bash
rsync -av \
  PC3_USER@PC3_IP:/ABSOLUTE_PATH/H2_project/log/h2-state/build-summary.txt \
  data/from-h2/first-test-001/logs/h2-state-build-summary.txt
```

这些命令不删除 PC3 原始数据。重做一次完整测试时，将说明中的 `first-test-001` 统一换成新的编号，例如 `first-test-002`；采集和报告工具会拒绝覆盖已有输出。
