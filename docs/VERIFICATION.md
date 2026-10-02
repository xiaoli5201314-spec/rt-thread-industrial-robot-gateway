# 主机验证记录

[返回 README](../README.md) | [构建与测试](BUILD_AND_TEST.md) | [架构与边界](ARCHITECTURE.md) | [协议文档](PROTOCOLS.md)

## 2026-10-02 单元测试

本记录对应本次强制重新构建后的实际运行结果，不复用旧二进制的历史说明。以下仅记录已核验的信息，不补造性能测量。

| 项目 | 本次记录 |
| --- | --- |
| 执行日期 | 2026-10-02 |
| 系统 | Ubuntu 22.04 |
| 编译器 | GCC 11.4 |
| 准确命令 | `make -B test` |
| 测试类型 | 主机 pthread 端口的 C 单元测试 |
| 调度模式 | `SCHED_OTHER(CFS)` |
| 用例组 | 117 |
| 断言总数 | 44095 |
| 失败断言 | 0 |
| 结果 | **PASS** |
| 跳过项 | 真实实时调度断言，见下文 |

复现本次命令时，从仓库根目录运行：

```bash
make -B test
```

根 [Makefile](../Makefile) 转发到 [gateway/Makefile](../gateway/Makefile)。`-B` 强制重新构建，`test` 构建并执行 `gateway/build/gw_tests`。显式选择 GCC 的推荐命令为 `make -B CC=gcc test`，但它不是本次报告的原始命令，应与记录区分。

## 优先级继承与跳过边界

发布前于 **2026-10-03** 在相同环境再次执行 `make -B test`，退出码 0，仍为 **117 个用例组、44095 条断言、0 失败**；真实实时调度断言仍在 CFS 模式下跳过。

[test_priority_inherit.c](../gateway/test/test_priority_inherit.c) 将机制层和真实调度层断言分开：

- 本次检查机制层：互斥锁竞争、持有者逻辑优先级提升、释放后恢复，以及启用/关闭继承的对照。
- 本次未检查真实实时调度层：继承开启时低优先级持锁者先于中优先级线程完成、关闭时的相反完成顺序，以及高优先级等待时间的对比断言。
- 运行于 `SCHED_OTHER(CFS)` 时，该调度层用例输出 `[SKIP]`。用例组计数仍包含这个标题，不能把 117 个用例组解读为每组全部调度分支都执行。
- 没有本次 `SCHED_FIFO` 运行记录、实时调度特权验证或目标 RT-Thread 时延测量；逻辑 PI 机制通过不等于真实调度性能通过。

## 这次 PASS 不证明什么

- 没有本次整机仿真、外部 TCP 对端或缺失 Python 工具的验收结果。
- 没有 RT-Thread BSP 编译、烧录、板端 DMA/ISR、CAN 控制器或真实设备互操作结果。
- 没有原理图、PCB、BOM、板型说明、真实 Flash 掉电保持或硬件保护试验。
- 不排除源码已指出的满长 TCP ADU 池容量、存储线程栈预算、在线参数下发、总线 FAULT 恢复和最新槽载荷回退问题。
- 本节的本地主机结果与远端 CI 分开记录；远端测试不替代 BSP 或真实设备验收。

测试范围和未覆盖路径详见 [BUILD_AND_TEST.md](BUILD_AND_TEST.md)。后续增加新的环境、协议对端或板端结果时，应另列准确命令、配置、输出与跳过项，不把本条主机记录扩展成未经执行的结论。

## 远端自动测试

2026-10-03 核对发布提交 `b64141f` 的 [GitHub Actions 记录](https://github.com/xiaoli5201314-spec/rt-thread-industrial-robot-gateway/actions/runs/37053451197)，状态为 `completed / success`。
工作流在 Ubuntu 22.04 执行 `make CC=gcc test`。
后续提交的实时状态以首页徽章和对应 Actions 记录为准；远端主机测试仍不提供 RT-Thread 板端时延或真实实时调度验收。
