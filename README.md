# H2 自主定位导航

目标平台：宇树 H2、MID-360（含内置 IMU）、PC3 NVIDIA Thor、Ubuntu 24.04 / ROS 2 Jazzy。

GitHub：[liuzhaoming5954/H2_project](https://github.com/liuzhaoming5954/H2_project)。主分支为 `main`，团队使用功能/修复分支提交 PR；协作约定见 [CONTRIBUTING.md](CONTRIBUTING.md)。

当前已实现并在 x86_64 / ROS 2 Jazzy 验证运动监护基础工程：5 个 ROS 包、公共消息、默认禁用的指令闸门、独立模拟监护、理想运动平台与构建工具。完整首版导航功能尚未完成，未做 Thor/H2 实机验收；进度见 [实现状态](docs/implementation_status.md)。

- [首版功能范围基线 V1.0](docs/v1_scope.md)：必交付能力、运行边界、失败/恢复行为、排除项和完成条件。
- [软件总体架构 V0.4](docs/software_architecture.md)：模块职责、数据流、坐标系、算法与部署原则。
- [本机开发与 H2 部署验证流程](docs/development_deployment.md)：长期开发及 A/B 部署设计；当前远程部署仅提供预览。
- [消息与运行契约](docs/interfaces.md)：已实现接口、会话、有效期与模拟边界。
- [本机工具](tools/README.md)：环境检查、构建测试、源码打包和部署预览。
- [定位建图依赖](dependencies/README.md)：固定版本的 Livox SDK2、驱动、FAST-LIO2 和 RTAB-Map，使用独立工作空间构建。
- [第一次上机逐步操作教程](docs/first_hardware_test_tutorial.md)
- [第一轮 H2 只读上机测试](docs/first_hardware_test.md)：环境清单、真实网络配置、原始数据检查/录制及独立 H2 被动状态采集。

首版覆盖已验收的室内平层建图、自动初定位/重定位、单点/多点导航、障碍处理、运动监护与数据记录。按范围基线实施与验收，具体依赖版本和实机参数在实现过程中收敛。

## 构建与验证

Ubuntu 24.04 首次配置可在交互终端执行 `bash scripts/install_ros_jazzy.sh`；脚本需要 sudo，安装工具链，不修改 H2 固件或服务。已安装 Jazzy 可直接运行：

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/h2ctl.py doctor --profile ros
python3 tools/h2ctl.py check --profile core --jobs 2
python3 tools/h2ctl.py check --profile ros --jobs 2
source install/ros/setup.bash
python3 tests/ros/test_motion_pipeline.py
python3 tests/ros/test_launch_modes.py
```

节点测试会创建仅 localhost 的独立 DDS 图，不加载 Unitree SDK。若运行环境禁止本机套接字，测试会失败，应在允许本机 DDS 通信的开发终端执行；不能将通信失败标为通过。

## 启动理想运动平台

```bash
source /opt/ros/jazzy/setup.bash
source install/ros/setup.bash
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$PWD/log/runtime"
ros2 launch h2_bringup system.launch.py mode:=sim
```

默认只发布静止的模拟里程计与禁用状态。运动需要显式服务使能、新鲜模拟许可及带会话/序号/原始有效期的指令；上述 smoke test 会在自己创建的图中演示这一流程和失效停车。当前不包含激光、碰撞检测、Nav2、完整步态或真实 H2 SDK，不能用于控制机器人。

`mode:=replay` 只验证隔离入口，不启动运动进程，也尚未集成 LIO/地图算法。`hardware_readonly` 需要显式的 MID-360 或 MID-360S 实机配置，只启动雷达采集；H2 状态由独立原生探针订阅。`hardware` 运动入口仍明确拒绝启动。模拟命令限制和机体高度均为测试参数，不是 H2 标定结果。只读模式命令和验收范围见 [首轮上机说明](docs/first_hardware_test.md)。
