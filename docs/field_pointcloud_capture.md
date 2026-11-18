# H2 现场点云采集教程

**交付目标**　用 H2 背包电脑 PC3 录下 MID-360S 的原始点云和内置 IMU，把完整文件夹交给建图人员。先做一次 2 分钟的小闭环，收到回放反馈后再采大范围路线。机器人由现场操作员用原厂方式操控，本教程的程序只负责采集传感器数据。

**这里的“闭环”指实际走回原位。** 选一个容易找准的位置和朝向，例如门槛旁的一处地面标记、面朝走廊尽头。录制开始和结束时都站回这个位置并朝同一方向。建图人员会检查两次经过同一扇门时墙面是否重合，以及计算出的轨迹离起点有多远。现有实机验证覆盖了 30 秒静止录制和离线回放，这次短路线用于验证移动效果。

## 1. 准备 PC3 和三个终端

| 终端 | 在哪台电脑上 | 用途 |
| --- | --- | --- |
| A | PC3 | 启动雷达驱动，采集期间保持运行 |
| B | PC3 | 检查数据、录制、核对结果、准备交付文件 |
| C | PC3 | 看录制日志，告诉操作员何时可以出发 |

三个终端可以是同一台 PC3 的三个窗口，也可以是连到它的三个 SSH 会话。**本教程中只有第 7 步的接收命令在开发机执行。** 一段路线的操作请保留同一个 B 窗口，其中的 `$route` 会一直指向本段数据。

在 **A** 检查已有环境。

```bash
cd ~/H2_project
hostname
uname -m
git rev-parse --short HEAD
ls /opt/ros/jazzy/setup.bash install/mapping/local_setup.bash install/ros/local_setup.bash \
   config/local/mid360s.json tools/readonly_capture.py
```

PC3 架构应为 `aarch64`，最后一条应列出全部五个文件。已经完成首次实机录制的 PC3 可以复用原安装、雷达配置和网线。新 PC3、缺文件或只有 `main` 的初始文档时，先按首次上机教程第 1～3 步取得 `codex/h2-readonly-first-test` 分支、构建并填写本地配置，再回到这里。[首次上机安装与配置](first_hardware_test_tutorial.md)

确认雷达已供电。继续在 **A** 读取当前配置并查看网口。

```bash
PYTHONPATH="$PWD/src/h2_bringup/launch" python3 -c \
  'import sys; from mid360_config import load_mid360_config; print(load_mid360_config(sys.argv[1]))' \
  "$PWD/config/local/mid360s.json"
read -r -p '输入上面显示的 lidar_ip，再按回车：' lidar_ip
ip route get "$lidar_ip"
```

核对三个值。配置里的 `lidar_ip` 对应现场雷达，路由输出里的 `dev` 对应雷达网线插入的网口，`src` 应等于配置里的 `host_ip`。若出现占位符、走错网卡或地址不一致，先按首次上机教程第 3 步核对配置和接线。

## 2. 启动驱动，准备录制环境

先在 **A** 执行下面整段，再到 **B** 执行同一段。每次新开 A 或 B 都要重新执行。

```bash
cd ~/H2_project
source /opt/ros/jazzy/setup.bash
source install/mapping/local_setup.bash
source install/ros/local_setup.bash
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
export ROS_LOG_DIR="$PWD/log/readonly"
mkdir -p "$ROS_LOG_DIR" "$PWD/data/site-001"
```

只在 **A** 启动一次驱动。出现主机和雷达地址后，保持这个窗口运行。接下来去 **B** 操作。

```bash
ros2 launch h2_bringup system.launch.py \
  mode:=hardware_readonly \
  lidar_config:="$PWD/config/local/mid360s.json"
```

## 3. 为两分钟小闭环创建目录，先检查 10 秒

在 **B** 执行。每段数据都有一个带时间的路线目录，重录时也会用新名字。

```bash
route="$PWD/data/site-001/loop-$(date +%Y%m%d-%H%M%S)"
seconds=120
max_mib=1024
reserve_mib=2048
mkdir "$route"
echo "$route"
df -h "$route"
```

记下打印的完整路径。看 `df` 输出的 `Avail`（可用空间），短路线建议至少留 **3 GiB**。如果 `mkdir` 提示目录已存在，再执行整段生成新名字。**只建到路线这一层**，下一步的 `capture` 子目录由录制程序创建。

在 **B** 运行检查。保持机器人静止。

```bash
python3 tools/readonly_capture.py inspect \
  --duration 10 --report "$route/inspect.json"
```

约 10 秒后，查看报告最外层的 `"ok": true`。如果为 `false`，先按第 9 步排查。如果为 `true` 且只有 `lidar.warnings.nonmonotonic_point_offsets` 警告，可以继续，实机录制中也观察到过这种帧内点序回退。

## 4. 开始录制，人工走一圈

1. 将 H2 停在选定标记，记住朝向。头顶雷达会跟随头颈运动，尽量维持自然、稳定的头姿。提前选一条能从容折返的路线，例如经过门框和走廊转角，再回到标记。可以由一人操控、另一人看终端和报时。
2. 在 **B** 打印本次日志的完整路径，复制终端显示的路径。

```bash
echo "$route/capture/recorder.log"
```

3. 在 **C** 执行下面两行，按提示粘贴刚复制的路径。此时会提示文件尚未创建，保持窗口打开，`tail -F` 会等文件出现后显示日志。

```bash
read -r -p '粘贴本次 recorder.log 的绝对路径：' record_log
tail -n +1 -F "$record_log"
```

4. 回到 **B** 执行录制命令。B 通常会安静地等到整段结束，录制进度看 **C**。

```bash
python3 tools/readonly_capture.py record \
  --duration "$seconds" \
  --output "$route/capture" \
  --config "$PWD/config/local/mid360s.json" \
  --max-size-mib "$max_mib" \
  --min-free-mib "$reserve_mib"
```

5. **C 看到两路都已订阅后，通知操作员开始计时。** 实机日志中的英文如下，两路订阅出现的先后顺序可能不同。较早出现的 `Recording...` 只说明录制器已启动。若 B 已退出且显示失败，留在原地，按第 9 步处理。

```text
Subscribed to topic '/livox/imu'
Subscribed to topic '/livox/lidar'
All requested topics are subscribed. Stopping discovery...
```

6. 起点先静止约 **15 秒**，再沿路线平稳行走、转弯。尽量在计时 **1 分 45 秒**前回到标记并恢复起始朝向，原地等 B 自行结束。提前到达可以多等一会儿。如果未能及时回到起点，记录实际终点，用更短的路线另录一段。
7. B 打印 JSON 并回到命令提示符后，这段录制结束。**只在 C 按 Ctrl+C** 退出日志查看。A 可以继续运行供下一段使用。B 的录制窗从录制器订阅就绪后开始，实际占用终端的时间会略长于 120 秒。

## 5. 在 PC3 核对这一段是否录全

在原来的 **B** 执行。第一条查看采集报告，第二条查看实际写入磁盘的 bag，并保存一份可直接阅读的摘要。

```bash
cat "$route/capture/report.json"
ros2 bag info "$route/capture/bag" | tee "$route/bag-info.txt"
```

| 核对位置 | 两分钟短路线应看到什么 |
| --- | --- |
| `report.json` | 最外层 `ok`、`completed_duration` 和 `recorder_ready` 均为 `true`，`failure` 为 `null`，`recorder_exit_code` 为 `0`，`bag.metadata_checks_passed` 为 `true` |
| `ros2 bag info` 的 `Duration` | 接近 120 秒 |
| `Topic information` 中的 `Count` | `/livox/lidar` 为 `livox_ros_driver2/msg/CustomMsg`，约 1,200 帧。`/livox/imu` 为 `sensor_msgs/msg/Imu`，约 24,000 条 |
| 实际路线 | 记录起步、回到起点和最终朝向，说明途中是否转头、停顿或遇到遮挡 |

计数按上次实测约 10 Hz 点云、200 Hz IMU 估算，允许启动和收尾有少量差异。请报上 **bag 里的两个实际 Count**。上次 30 秒 bag 存有 6,002 条 IMU，同一份报告里的 `imu.message_count` 只有 3,115，那是旁路观察程序自己处理的数量。若时长明显不足、缺话题或数量明显偏低，保留这段并标出异常。

## 6. 写路线说明，准备交付

在 **B** 打开这一段的记录文件，照下面示例填写实际情况。操作员记得的起终点、朝向和重复经过的位置，对判断地图是否拼准很重要。

```bash
nano "$route/route.txt"
```

```text
场地和路线编号：一层，短闭环 01
起点和朝向：东侧会议室门外地面标记，面向电梯
途经：经过会议室门框，在第一个转角左转，再沿原路返回
终点和朝向：回到原标记，面向电梯。起终点各静止约 15 秒
与其他段重复的位置：本段是第一段
头部及异常：头部朝前。约第 50 秒有人经过雷达前方
现场检查：按第 5 步填写实际时长、两路 Count、报告 ok 和异常
```

Nano 中按 Ctrl+O、回车保存，再按 Ctrl+X 退出。检查该段目录，然后生成传输校验清单。此后若补写了路线说明，需要再执行一次生成命令。

```bash
ls -Rh "$route"
(cd "$route" && find . -type f ! -path './SHA256SUMS' -exec sha256sum {} + > SHA256SUMS)
echo "$route"
```

交付的是最后一行打印的**整个路线目录**。其中 `capture/bag/metadata.yaml` 与同目录下的**全部 `.db3` 文件**要一起保留。其他随附文件包括外层的 `inspect.json`、`bag-info.txt`、`route.txt`、`SHA256SUMS`，以及 `capture/` 中的 `report.json`、`recorder.log` 和 `mid360-config.json`。工具也会复制已有的 `navigation.repos` 和 `build-summary.txt`。

## 7. 回传文件，先让建图人员检查短闭环

**开发机能通过 SSH 登录 PC3 时**，把 B 打印的路线绝对路径，以及可登录 PC3 的用户名和管理地址发给接收人。SSH 地址使用现场连接 PC3 的地址，雷达的 `lidar_ip` 仅用于接传感器。接收人在开发机上打开现有的 `H2_project` 文件夹，右键选择“在终端中打开”。先执行 `pwd` 确认位于项目根目录，再执行下面整段。`df` 显示的可用空间应能容纳这一整段数据。

```bash
dest="$PWD/data/from-h2/site-001"
mkdir -p "$dest"
df -h "$dest"
read -r -p 'PC3 SSH 登录目标（用户名@地址）：' pc3_login
read -r -p 'PC3 上本段路线的绝对路径：' pc3_route
pc3_route="${pc3_route%/}"
rsync -avP "${pc3_login}:${pc3_route}" "$dest/"
```

源路径保留路线目录名，开发机上应出现 `data/from-h2/site-001/loop-日期-时间/`。若传输中断，保留源文件和接收目录，重新运行同一条 `rsync`。等待成功后在**开发机**核验。

```bash
received="$dest/$(basename "$pc3_route")"
(cd "$received" && sha256sum -c SHA256SUMS)
```

每行应为 `OK`（中文终端可能显示“成功”），尤其是 `metadata.yaml` 和各个 `.db3`。出现 `FAILED` 或缺文件时，在同一个开发机终端执行下面两行，按内容比较并重传，然后再次校验。接收人确认校验通过前，保留 PC3 原件。

```bash
rsync -avPc "${pc3_login}:${pc3_route}" "$dest/"
(cd "$received" && sha256sum -c SHA256SUMS)
```

**两台电脑无法互联时**，可以把整个路线目录复制到移动硬盘，或用团队约定的文件传输渠道发送给接收人。接收人将目录原样放入开发机项目根目录下的 `data/from-h2/site-001/`，在该路线目录打开终端运行 `sha256sum -c SHA256SUMS`。原始现场数据通过该渠道交付，公开 GitHub 仓库用于软件和教程。

**第一次只交付短闭环。** 把路线目录名、操作员描述的实际闭合情况和校验结果一起报给建图人员。由建图人员回放点云和 IMU，检查移动中两路数据是否连续、转角处墙面和门框是否重影，以及首末位姿与现场描述是否一致。拿到“可以扩大采集”或“请重录这段”的反馈后，按反馈执行。现场报告的 `ok` 只覆盖录制和双流检查，地图质量由回放判断。

## 8. 扩大到走廊、房间和整层

1. 先在平面图或纸上划路线，如“A01 大厅—东走廊—大厅”“A02 东走廊—会议室—东走廊”。每条尽量连续录完，安排从能辨认的标记出发并返回。通过门口时兼顾门内外，平稳经过转角。相邻两段要**重复走过一小段有门框、墙角等共同特征的路线**，在各自的 `route.txt` 写明同一处地标和行进方向。只在同一地点停一下，留给后续拼接的共同几何信息较少。
2. 以下以**每段 10 分钟**为例。在 **B** 重新进入仓库，设置新编号和录制参数。当天新开终端时先执行第 2 步的环境命令，并确认 A 的驱动仍在运行。

```bash
cd ~/H2_project
zone=A01
route="$PWD/data/site-001/area-${zone}-$(date +%Y%m%d-%H%M%S)"
seconds=600
max_mib=4096
reserve_mib=4096
mkdir "$route"
echo "$route"
df -h "$route"
```

3. 根据 `Avail` 检查磁盘。上次实测 30 秒约 117 MiB，按同样的数据率，**2 分钟约 0.46 GiB，10 分钟约 2.3 GiB**。10 分钟示例建议开始前可用空间至少 **8 GiB**。每录下一段都要重新看剩余空间，包括开发机用于接收的空间。
4. 保留上面设置的 600 秒参数，从**第 3 步的 `inspect` 命令**开始，接着执行第 4～7 步。看到双订阅后先静止约 15 秒，留出回程时间，争取在计时 **9 分 45 秒**前回到终点并保持静止。检查表改为 **约 600 秒、6,000 帧点云、120,000 条 IMU**。每段都写路线说明、生成校验清单并回传。
5. 下一段从本节第 2 项重新开始，把 `zone=A01` 改为 `zone=A02`，再依次编号。现场因中断分成两包时，记下中断点，从能重复覆盖上一段末尾的位置重新开始。每个 bag 各有自己的起点，跨包对齐留给后续建图分析。
6. 全部录制完成、B 已回到命令提示符后，在 **A** 按 Ctrl+C 停止驱动。将区域编号、对应目录、已覆盖和遗漏的房间发给接收人，逐段确认收齐。

`--max-size-mib` 是本次会话的大小阈值，`--min-free-mib` 是要留给磁盘的可用空间。碰到任一阈值，工具会提前停止并保留部分数据，不会自动开启下一包。单次时长上限为 3,600 秒，较长任务可按以上路线分段。

## 9. 遇到问题时怎么处理

| 现场现象 | 处理 |
| --- | --- |
| `--output must be an absolute, new session directory` 或提示报告已存在 | 确认在 B，`echo "$route"` 显示本次完整路径。保留旧目录，短路线重新执行第 3 步的建目录命令，区域路线从第 8 步设置新目录。只预建路线目录，让工具创建 `capture/`。 |
| 检查收不到双流，或 C 始终看不到订阅完成 | 看 B 是否已退出并记录报错，核对 C 路径与 B 打印的一致。检查 A 的日志、雷达供电、网线、路由和两个 ROS 终端的环境。修复后用新的路线目录重新检查。 |
| 旧版工具将 `nonmonotonic_point_offsets` 列在 `errors` 并使检查失败 | 记下 PC3 的提交号并保存报告。按首次上机教程第 1 步更新到 `codex/h2-readonly-first-test`，再建新目录复查。当前工具会分别报告帧内点序警告和消息时间戳错误。 |
| 录制中断、磁盘不足、`ok: false`，或没有 `metadata.yaml` | 保留已有文件、`report.json`、`recorder.log` 和 A/B 的报错。报告缺失时记录终端输出。标记为“未完整”，先向建图人员传可用的诊断，处理空间或连接问题后用新目录重录。 |
| 录完才发现走错路线、偏离起点或支架变动 | 写清异常位置和大致时间，保留原包。检查安装后，重新选择能准时返回的路线，用新目录补录。 |
