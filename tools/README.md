# 本机工具

`h2ctl.py` 只使用 Python 3 标准库，无需额外 Python 安装步骤。它不会连接机器人或发送运动命令。

在仓库根目录运行：

```bash
python3 tools/h2ctl.py doctor --profile core
python3 tools/h2ctl.py doctor --profile ros --json
python3 tools/h2ctl.py check --profile core
python3 tools/h2ctl.py check --profile ros
python3 tools/h2ctl.py bundle --output artifacts/h2-source.tar.gz
python3 tools/h2ctl.py deploy --target-config tools/target.example.json --preview
```

- `doctor` 检查当前环境，不安装或修改系统。`ros` 模式要求已经加载 ROS 2 Jazzy 环境；缺少必需工具时退出码为 2。
- `check --profile core` 实际执行根 CMake、构建和 CTest，目录为 `build/core`。`ros` 模式实际执行 colcon 的构建、测试及结果检查，使用独立的 `build/ros`、`install/ros`、`log/ros`。可用 `--jobs N` 限制并行度。检查失败不会继续报告通过，二者都不代表 ARM64 或实机验收。
- `bundle` 只收集明确的项目源码目录和元数据，排除 Git 元数据、构建产物、本地配置、记录数据和缓存。符号链接与非普通文件会被拒绝；单文件限制为 64 MiB。已有输出文件不会覆盖，请更换文件名。归档中的 `MANIFEST.json` 包含每个文件的 SHA-256、大小、可执行权限及整个文件清单的快照标识；Git 提交号只是参考，不能代替未提交源码的快照标识。
- `deploy --preview` 只读取目标 JSON 并展示固定 A/B 部署计划，**没有远程执行或版本激活功能**。省略 `--preview` 会明确失败。示例地址和用户名必须在实机接入后替换；实际本机配置放在 `config/local/` 等不纳入源码的位置。

离线工具测试：

```bash
python3 -m unittest discover -s tests/tools -v
```

ROS 环境检查和构建需先加载 `/opt/ros/jazzy/setup.bash` 或实际的 Jazzy 安装环境；工具按 `PATH` 查找 CMake/colcon，不依赖某个临时安装路径。
