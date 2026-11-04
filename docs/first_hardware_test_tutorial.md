# H2 第一次上机操作教程

这一轮在 H2 的 PC3 算力背包上验证两件事：PC3 能否持续收到 MID-360S 的点云和内置 IMU，以及能否把两路数据录成一个可回放的 bag。测试时让机器人保持静止，启动的实机程序只有雷达驱动和采集工具。需要核对正式验收范围或采集 H2 自身的状态时，查看[首轮测试说明](first_hardware_test.md)。

- **终端 A** 在 PC3 上，用来下载、构建、填配置，最后运行雷达驱动。
- **终端 B** 也在 PC3 上，用来检查和录制。可以打开第二个 SSH 会话。
- **开发机终端** 在测试结束后使用，用来取回文件。

以下假设仓库在 PC3 的 `~/H2_project`。如果你克隆到别处，把命令中的这个路径换成实际路径。每段命令运行完，先对照本步的预期结果，再继续。

## 1. 在 PC3 上取得代码

在终端 A 运行：

```bash
hostname
uname -m
cat /etc/os-release
```

确认这是 PC3 的终端，架构为 `aarch64`，系统是 Ubuntu 24.04。若显示 `x86_64`，先确认是否还在开发机终端。

**第一次下载**时运行：

```bash
cd ~
git clone --branch codex/h2-readonly-first-test --single-branch \
  https://github.com/liuzhaoming5954/H2_project.git H2_project
cd ~/H2_project
```

仓库是公开的，克隆无需 GitHub 登录。如果 `~/H2_project` 已经存在，改用以下方式。先运行前两行检查工作区。若 `git status --short` 列出文件，先核对这些改动。确认工作区干净后，再运行后两行。

```bash
cd ~/H2_project
git status --short
git fetch origin refs/heads/codex/h2-readonly-first-test
git switch --detach FETCH_HEAD
```

然后运行：

```bash
git rev-parse --short HEAD
ls src/h2_bringup/config/mid360s.example.json
mkdir -p data/first-test-001 log/readonly config/local
python3 tools/readonly_environment.py --report data/first-test-001/environment-before.json
```

第一行输出的是本次测试的提交号。第二行应找到 MID-360S 模板。如果模板不存在，先检查下载的分支。若报告已经存在，保留旧报告，为本轮选一个新编号，例如 `first-test-002`，并把后面命令里的编号一起换掉。

## 2. 安装依赖并构建

继续在终端 A。先检查 ROS 2 Jazzy：

```bash
ls /opt/ros/jazzy/setup.bash
```

如果文件不存在，在 PC3 上执行 `bash scripts/install_ros_jazzy.sh`。有文件就跳过 ROS 安装。第一次在 PC3 准备项目的 Mapping 依赖时，执行：

```bash
bash scripts/install_mapping_dependencies.sh
```

安装脚本可能提示输入 PC3 的 sudo 密码，终端输入密码时通常没有回显。如果报某个固定版本的软件包找不到，保存报错，先核对 PC3 的软件源和依赖版本。

接着在仓库根目录按顺序运行。下载和编译可能需要等待，每条成功后再执行下一条。

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/h2ctl.py doctor --profile ros
bash scripts/fetch_navigation_sources.sh
bash scripts/build_mapping_dependencies.sh --jobs 2
python3 tools/h2ctl.py check --profile ros --jobs 2
```

`source` 让当前终端找到 ROS 工具。第三条下载锁定的依赖源码，第四条在 PC3 本机编译，最后一条构建项目的 ROS 包并运行软件测试。看到 `ros checks passed` 后再继续。若构建报错，保留终端报错及仓库下的 `log/mapping/` 或 `log/ros/`。这些命令通过后，下一步才是接入真实雷达数据。

## 3. 找到 PC3 接雷达的网卡地址

确认 MID-360S 已供电，网线接到 PC3 的雷达网口。在终端 A 运行：

```bash
ip -br -4 address
read -r -p '请输入 MID-360S 的固定 IPv4，然后按回车: ' h2_lidar_ip
ip route get "$h2_lidar_ip"
```

输入现场已确认的雷达固定地址。最后一条命令的输出里，`dev` 后面是 PC3 打算使用的网卡，`src` 后面是 PC3 打算使用的本机 IPv4。对照实际接线确认 `dev` 是雷达网口。若选中了 Wi-Fi、管理网口或经默认网关出去，先检查雷达网口的 IPv4 和掩码。路由输出用来确认 PC3 选了哪张网卡，能否收到数据在第 5 步验证。

在仓库根目录创建本地配置，随后用编辑器打开 `config/local/mid360s.json`。

```bash
cp -n src/h2_bringup/config/mid360s.example.json config/local/mid360s.json
nano config/local/mid360s.json
```

只填写这两个地址。JSON 中保留地址两边的双引号。

| 在文件里找到 | 换成 |
| --- | --- |
| `REPLACE_WITH_MID360S_IPV4`，位于 `lidar_configs` 里的 `ip` | 雷达自己的固定 IPv4 |
| `REPLACE_WITH_PC3_SENSOR_IPV4`，位于 `Mid360s` 里的 `host_ip` | 上一步确认的 PC3 雷达网口 IPv4，即 `src` |

两处地址必须不同。保留模板的大小写、括号、端口和零外参。若文件已经填写过，`cp -n` 会保留原文件，打开后直接核对 `ip` 和 `host_ip` 的现有值。使用 Nano 时按 Ctrl+O、回车保存，再按 Ctrl+X 退出。也可以用你熟悉的编辑器。保存后运行：

```bash
PYTHONPATH="$PWD/src/h2_bringup/launch" python3 -c \
  'import sys; from mid360_config import load_mid360_config; print(load_mid360_config(sys.argv[1]))' \
  "$PWD/config/local/mid360s.json"
```

成功时会打印配置路径、`host_ip` 和 `lidar_ip`。如果报占位符或 JSON 格式错误，回到编辑器修正后再运行。

## 4. 终端 A 启动雷达驱动

```bash
cd ~/H2_project
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

驱动会一直在前台运行。保留终端 A，暂时别按 Ctrl+C。启动信息应显示本机和雷达地址。若提示找不到包，检查第 2 步构建结果和三条 `source` 命令。若 SDK 报初始化失败，记录原始报错并回到第 3 步核对网口。即使节点还在运行，也以终端 B 实际接收的数据为准。

## 5. 终端 B 先检查 10 秒，再录制 30 秒

新开一个 **PC3** 终端。每个终端都要加载相同的 ROS 环境，才能在 PC3 本机发现同一组话题。

```bash
cd ~/H2_project
source /opt/ros/jazzy/setup.bash
source install/mapping/local_setup.bash
source install/ros/local_setup.bash
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
export ROS_LOG_DIR="$PWD/log/readonly"
python3 tools/readonly_capture.py inspect \
  --duration 10 \
  --report "$PWD/data/first-test-001/livox-inspect.json"
```

等待命令打印报告。看到 `"ok": true`，说明检查窗口内收到了符合本轮要求的点云和 IMU。若 `ok` 为 `false`，先检查终端 A 是否仍在运行、两个终端是否都设置了域 81，以及雷达供电、网口、地址和驱动日志。需要重跑时换一个新的报告文件名，保留之前的诊断。如果 `lidar.warnings` 只有 `nonmonotonic_point_offsets` 且 `ok` 为 `true`，可以继续录制。它统计一帧内偏移的回退次数，录包会保留收到的原始点序。

通过后在终端 B 运行：

```bash
python3 tools/readonly_capture.py record \
  --duration 30 \
  --output "$PWD/data/first-test-001/livox-stationary" \
  --config "$PWD/config/local/mid360s.json" \
  --max-size-mib 512 \
  --min-free-mib 2048
```

保持机器人静止，等待命令自行结束。程序要先等两路数据和录制器订阅就绪，才开始计时，所以实际等待可能超过 30 秒。如果输出目录已经存在，换一个新目录名。若提示空间不足，用 `df -h .` 检查仓库所在分区。

## 6. 确认结果并结束采集

仍在终端 B：

```bash
cat data/first-test-001/livox-stationary/report.json
ros2 bag info data/first-test-001/livox-stationary/bag
```

录制成功时，报告应有 `"ok": true`，bag 信息里 `/livox/lidar` 和 `/livox/imu` 的消息数都应大于零。核心数据文件是 `bag/metadata.yaml` 和 `bag/bag_0.db3`。报告失败或 bag 不完整时，把已有的报告、配置快照和 `recorder.log` 一起保留。文件分布见[产物目录树](first_hardware_test.md#6-核对-pc3-上的测试产物)。

录制命令退出后，回终端 A 按 Ctrl+C 停止驱动。如果还要单独读取 H2 机身或躯干状态，按[被动状态采集步骤](first_hardware_test.md#5-可选h2-被动状态采集)使用可选探针。雷达录包里的 IMU 来自 MID-360S 本身。

## 7. 把结果带回开发机

先在 PC3 的仓库目录运行 `pwd`，记下输出的绝对路径。然后回到**开发机**终端，进入开发机上的项目目录。运行以下命令时，依次输入 PC3 的 SSH 用户名、开发机能连接的 PC3 地址和刚记下的路径。SSH 地址用于登录 PC3，和第 3 步的雷达地址分别核对。

```bash
read -r -p 'PC3 SSH 用户名: ' h2_pc3_user
read -r -p 'PC3 SSH 地址: ' h2_pc3_ip
read -r -p 'PC3 仓库绝对路径: ' h2_pc3_repo
mkdir -p data/from-h2/first-test-001/logs
rsync -av "$h2_pc3_user@$h2_pc3_ip:$h2_pc3_repo/data/first-test-001/" \
  data/from-h2/first-test-001/
rsync -av "$h2_pc3_user@$h2_pc3_ip:$h2_pc3_repo/log/readonly/" \
  data/from-h2/first-test-001/logs/readonly/
rsync -av "$h2_pc3_user@$h2_pc3_ip:$h2_pc3_repo/log/mapping/build-summary.txt" \
  data/from-h2/first-test-001/logs/mapping-build-summary.txt
```

先确认第一个 `rsync` 成功。回传结果在开发机仓库的 `data/from-h2/first-test-001/`，可用 `ls data/from-h2/first-test-001/` 查看。如果构建阶段失败，对应构建记录可能尚未生成，先保留终端错误和已有日志。状态探针的可选日志见[数据回传清单](first_hardware_test.md#7-将数据带回开发机)。

数据回传完成后，可在开发机按[静态 LIO 回放说明](lio_static_replay.md)验证初始化和静止轨迹。
