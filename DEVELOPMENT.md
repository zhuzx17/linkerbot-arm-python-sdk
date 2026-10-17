# 开发与验证

贡献流程见[Linkerbot 贡献指南](https://github.com/linker-bot/.github/blob/main/CONTRIBUTING.md)；以下为本项目的开发与验证要求。

- arm 功能在 `linker-bot/linkerbot-arm-python-sdk` 维护，ROS2 消息/服务/启动配置在 `linker-bot/linkerbot-arm-ros2-sdk` 维护；默认分支均为 `main`。
- 公共导入路径和已有函数调用保持兼容，包括现有集成使用的 `_set_angles` 与 motor 接口。
- 新增功能优先用新方法或可选参数；修改已有签名前应提供明确迁移方案，不能直接重新生成基线掩盖破坏性变更。
- `tests/compat/upstream_api.json` 固定初始来源。`tools/check_upstream.py` 用于确认初次提取，
  后续有意修改实现后它可能报告差异；日常兼容性门禁使用 pytest 中的 API 测试。
- 共享通信代码与完整 SDK 的修复通过独立 commit 审阅、回归和按需移植，不自动复制覆盖。
- transport 使用当前 `main` 源码，不固定旧版本或提交。升级前验证 arm 接口、收发、关闭、
  ROS2 adapter 和独立 wheel，并同步安装文档；本地必要检查通过后提交。
- 两个 CAN 后端的源码安装清单见 `requirements/transport.txt`；验收使用源码安装。
  默认后端为 CANTransport，dispatcher 按 [说明](docs/TRANSPORT.md) 显式选择。
- CAN/CAN FD 后端实现和测试由 transport 维护；arm 保留兼容层与消息模型的测试，
  包括既有异常、回执和关闭约定。升级时也验证与 hand 的共存安装及共享厂商库生命周期。
- `docs/` 维护安装、快速开始、API 和用户迁移说明。临时 TODO、审查与过程记录放在
  本地 `.local-notes/`，该目录不纳入 Git 和发行包，用户文档不链接到该目录。

```bash
# 先按 docs/INSTALLATION.md 安装当前 transport 源码。
python -m pip install -e '.[kinetix,test,dev]'
python -m pytest
python -m build
```

测试默认排除硬件和交互用例。需要实际机械臂时，明确指定型号、CAN 和 marker，例如：

```bash
CAN_INTERFACE=can0 ARM_SIDE=left python -m pytest tests/arm/a7 -m 'hardware and not interactive'
```

修改打包或 URDF 后，必须在源码目录之外安装 wheel 验证导入和八个模型的加载。
CI 使用 `tools/check_installed.py` 检查该项，并确认 `py.typed` 与许可证进入安装包。
修改 ROS2 接入后，在安装新 SDK 的环境运行 ROS2 仓库 `tests/`；带 rclpy 的环境会额外运行真实节点测试。
可运行 `python tools/check_ros2_adapter.py ../linkerbot-arm-ros2-sdk` 检查真实 adapter 的调用契约（路径按实际 checkout 调整）；
该工具直接使用生产 loader 加载 `linkerbot_arm`，不创建旧命名空间别名；
适配器契约检查不能替代实际节点和设备运行验证。
发布前记录 Python、NumPy、SciPy、Pinocchio 版本以及真机验收结果，并更新 CHANGELOG。

## GitHub CI

PR 与 `main` 更新执行 Ruff lint/format、ShellCheck、actionlint，以及 Linux/Windows 上的
Python 3.10、3.13 离线测试；每周补测 3.11、3.12。测试安装真实 Pinocchio 并检查导入，
沿用现有配置排除 `hardware` 和 `interactive` 用例。独立打包任务构建 sdist、wheel，
在源码目录之外验证 wheel 和八个 URDF。每周及手动运行还会使用当前 SDK 测试最新
ROS2 wrapper 的 Humble/Jazzy Docker 环境。Transport 从 GitHub `main` 安装，
日志记录实际源码提交，不访问 Gitea。

PR 与 `main` 更新同时运行 Transport 中已有的双手双臂联合测试，沿用相同的
Linux/Windows、Python 矩阵。Arm 使用本次工作流提交（PR 使用合并结果），
Transport、Hand 在本轮联合矩阵内先解析为固定提交，再由全部联合任务复用。
测试运行真实 SDK 和 Pinocchio，只模拟四路虚拟 CAN 另一端的设备；每个任务保存
三个源码版本及 JUnit，验证并发收发、同 ID 隔离、单通道失联重建与背压隔离。

Linux 使用 PyPI 的 `[kinetix]` 依赖；Windows 沿用安装文档的 Conda 路线，
通过 micromamba 创建独立环境，安装 Conda Forge `pinocchio=4.1.0` 和基础 SDK 的
`[test]` 依赖，不混装没有 Windows wheel 的 PyPI `pin`。两平台执行相同的完整离线
测试选择并保存 JUnit/覆盖率，记录实际数值库版本；不额外跳过 Windows 失败用例。
这不代表厂商 Windows 驱动、真实 CAN 或机械臂动作已经验证。

CAN FD 兼容测试显式选择 Linux/Windows ABI，在两种主机上都覆盖参数、错误转换、
并发关闭和失败重试；Linux `LibCANbus_Init/Exit` 用例使用 Linux fake。
真实中断测试在 Linux 发送 SIGINT，在 Windows 新建独立控制台、显式启用 Ctrl+C 接收后发送事件，
并验证重复中断时先完成工作线程恢复再关闭总线；另用待处理 Python 中断覆盖阻塞等待。
工作线程清理采用有限等待片段，让挂起的中断有机会执行，等待超时本身不表示清理完成。
两条 CAN 发送路径用 `perf_counter` 计算短间隔，测试保持原间隔门限，并用粗粒度
单调时钟跳变复现发送节拍问题。Windows 上不使用会向系统时钟节拍取整的
`Event.wait` 补足 300 微秒发送间隔：Python 3.11+ 使用高分辨率 `sleep`，
Python 3.10 使用独立的 Windows 高分辨率等待计时器（Windows 10 1803+），
每次等待后释放句柄，不改变全局计时器分辨率；其他平台仍使用可中断等待。
七电机回归同时保持位置、速度和力矩的 30ms 轮询，并验证连续配置写入、反馈更新和
线程退出。收发元数据、业务超时、FIFO 顺序和配置写入的一秒截止时间保持原约定。
只补足物理发送后的剩余时间，已关闭连接或已到期时直接返回。
Windows 短等待期间的关闭请求至多额外请求剩余 300 µs，操作系统仍可能延后调度。
本地 Linux 单测覆盖分支选择、粗粒度 Event 等待复现、原生 API 错误和句柄清理；
这些替身测试不能替代 Windows runner 对真实计时器与并发轮询吞吐的验证。

复现 CI 工具和报告配置（先按安装文档准备 Transport）：

```bash
python -m pip install -r requirements/ci-lint.txt -r requirements/ci-test.txt
python -m pip install -e '.[kinetix,test]'
ruff check .
ruff format --check .
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -p pytest_timeout -p pytest_cov.plugin \
  --timeout=60 --cov=linkerbot_arm --cov-report=term-missing:skip-covered \
  --cov-report=xml:reports/coverage.xml --junitxml=reports/pytest.xml
```

显式加载超时和覆盖率插件，避免系统第三方插件干扰。覆盖率先记录基线，不设置最低阈值。
`reports/` 与 `dist/` 作为 Actions artifacts 保留 14 天；CI 不发布包，也不连接机械臂。

## 在个人 fork 验证 CI

Actions 的手动运行入口支持 `dependency_owner` 和 `dependency_ref`：默认仍使用
`linker-bot` 依赖，以及原有的 main/配对提交规则。跨仓库改动可先将 Transport、
Hand/Arm Python SDK 和两个 ROS 仓库推送到个人 fork 的同名测试分支，再统一指定
个人账号与该分支，避免混用上游尚未更新的测试脚本。日志记录实际使用的源码提交。

联合任务始终测试本次 Arm 工作流的代码；上述参数仅选择其他仓库的依赖。
所选 Transport 提交必须包含 `.github/actions/four-devices` 共享执行步骤；首次合入时
先合入 Transport，再合入 Hand/Arm 工作流。

新 fork 的工作流需要启用 Actions，且默认分支上需要包含工作流文件，才能手动运行；
可以仅将个人 fork 的默认分支设为测试分支。上游默认分支不受影响。

手动运行时设置 `full_matrix=true`，会补齐 Python 3.11、3.12；ROS/SDK 消费者任务也会运行。
