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
修改 ROS2 接入后，在安装新 SDK 的环境运行 ROS2 仓库 `tests/`；带 rclpy 的环境会额外运行真实节点测试。
可运行 `python tools/check_ros2_adapter.py ../linkerbot-arm-ros2-sdk` 检查真实 adapter 的调用契约（路径按实际 checkout 调整）；
该工具直接使用生产 loader 加载 `linkerbot_arm`，不创建旧命名空间别名；
适配器契约检查不能替代实际节点和设备运行验证。
发布前记录 Python、NumPy、SciPy、Pinocchio 版本以及真机验收结果，并更新 CHANGELOG。

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
