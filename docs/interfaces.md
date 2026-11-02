# 已实现消息与运行契约

更新日期：2026-11-03。适用当前运动监护基础工程，不代表完整 V1 导航接口已经冻结。

## 包与进程

| 包 | 实现 |
| --- | --- |
| `h2_interfaces` | MotionCommand、MotionPermit、MotionStatus 和 SetMotionEnabled |
| `h2_core` | 无 ROS 依赖的 C++17 指令/许可/会话与限幅核心 |
| `h2_runtime` | C++ `motion_guard` 与 `mock_platform`，均未加载 SDK |
| `h2_supervisor` | Python `supervisor`；只实现模拟健康策略，hardware 模式不发有效许可 |
| `h2_bringup` | sim/replay 入口；hardware/hardware_readonly 尚未接入且明确拒绝 |

## ROS 接口

| 名称 | 类型 | 发布/处理方 → 消费方 |
| --- | --- | --- |
| `/motion/command` | `h2_interfaces/msg/MotionCommand` | 测试客户端，未来导航源 → motion_guard |
| `/motion/permit` | `h2_interfaces/msg/MotionPermit` | supervisor → motion_guard |
| `/motion/safe_cmd` | `h2_interfaces/msg/MotionCommand` | motion_guard → mock_platform；保留原始创建时间与有效期 |
| `/motion/status` | `h2_interfaces/msg/MotionStatus` | motion_guard → supervisor/mock/观测工具 |
| `/motion/set_enabled` | `h2_interfaces/srv/SetMotionEnabled` | 显式操作 → motion_guard |
| `/state/odom` | `nav_msgs/msg/Odometry` | mock_platform → supervisor/观测工具 |
| `/state/odom_body_3d` | `nav_msgs/msg/Odometry` | mock_platform → 观测工具 |
| `/tf` | TF2 | mock_platform 发布 `odom→base_footprint→base_link` |

运动接口采用 reliable QoS；command/permit/safe_cmd 队列深度 1，status 深度 10。模拟 odometry 深度 10。所有速度单位为 m/s、rad/s，角度为 rad；运动消息 frame 必须匹配 `base_footprint`，odom 的父 frame 为 `odom`。本次 mock 的高度为可配置的理想值，不是 H2 尺寸标定。

## 使能与失效

1. 启动时输出零速，状态禁用。SetMotionEnabled(true) 创建新会话并等待第一条许可，不能自行授权运动。
2. 首条许可必须在 pending_timeout_s 内到达，且签发者、会话、frame、时间、有效期与序号有效；否则不能进入已使能状态。默认待许可上限 1 s。
3. 模拟 supervisor 同时检查 odom 和 guard 状态的接收时间/采集时间新鲜度、frame、有限值及机体姿态合法性，再签发短许可；它没有实机健康授权能力。
4. 指令 source 默认为 `navigation`；许可 source 默认为 `supervisor`，两者是不同身份。每种消息在同会话内序号严格递增，不允许回绕复用。
5. `header.stamp` 是原始创建时间，validity 从它起算。边界检查 ROS 时间年龄后转换到单调时间截止，后续重发不刷新截止；中间节点不得重盖时间戳。
6. 指令与许可分别到期；任一已生效项到期后锁止零速，续发消息不能复活旧会话，需显式重新使能得到新会话。已使能但尚无首指令时保持零速，许可仍须续期。
7. `SetMotionEnabled(false)` 或当前会话零速且 validity=0 的显式停止撤销运动。许可 enabled=false 同样撤销；旧会话/错误来源的普通数据不能覆盖有效的新会话。
8. 只允许前进和绕 z 转向，禁用侧移/倒退及其他轴。拒绝 NaN/Inf；有限的超限前进/转向值被限制。模拟端点再次检查状态、会话、授权序号、原始 TTL 和速度上限，不能靠重复 safe_cmd 延长运动。

sim.yaml 当前前进上限 0.2 m/s、转向上限 0.3 rad/s、guard 最大指令/许可有效期各 0.5 s；supervisor 以 20 Hz 发 0.25 s 许可。这些是本机模拟测试起点，禁止据此宣称 H2 运动能力或停止距离已验证。

当前尚未实现任务管理、速度平滑、Collision Monitor、硬件控制权接管或实体急停集成。`h2_core` 提供撤销原因与锁止逻辑；真实接管/急停状态必须在后续 SDK 适配中接入并实测。该软件边界也不是 DDS 身份认证或安全认证机制。

## 模拟与回放

sim 启动 mock、guard 和模拟 supervisor，默认无运动；不包含真实雷达、LIO、障碍物或步态。replay 本次只保留不加载运动进程的隔离入口，真实 bag 选择、算法链与时间控制在后续实现。

单个运行图中每条 TF 仅有一个发布者；真实估计链接入后，不允许同时保留 mock 的同名 odom/TF。模拟/回放与硬件模式不共用 SDK writer。
