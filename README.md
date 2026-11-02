# H2 自主定位导航

目标平台：宇树 H2、MID-360（含内置 IMU）、PC3 NVIDIA Thor、Ubuntu 24.04 / ROS 2 Jazzy。

GitHub：[liuzhaoming5954/H2_project](https://github.com/liuzhaoming5954/H2_project)。主分支为 `main`，团队使用功能/修复分支提交 PR；协作约定见 [CONTRIBUTING.md](CONTRIBUTING.md)。

当前阶段：首版功能范围已固定，进入详细设计和工程实现准备；尚无导航源码、编译结果或实机验收结果。

- [首版功能范围基线 V1.0](docs/v1_scope.md)：必交付能力、运行边界、失败/恢复行为、排除项和完成条件。
- [软件总体架构 V0.4](docs/software_architecture.md)：模块职责、数据流、坐标系、算法与部署原则。
- [本机开发与 H2 部署验证流程](docs/development_deployment.md)：仿真/回放分层、SSH 同步、Thor 构建、A/B 发布与回滚；工具入口尚待实现。

首版覆盖已验收的室内平层建图、自动初定位/重定位、单点/多点导航、障碍处理、运动监护与数据记录。按范围基线实施与验收，具体依赖版本和实机参数在实现过程中收敛。
