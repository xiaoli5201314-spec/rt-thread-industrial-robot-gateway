# 构建、测试与验证范围

[返回 README](../README.md) | [架构与边界](ARCHITECTURE.md) | [协议文档](PROTOCOLS.md) | [验证记录](VERIFICATION.md)

本文给出当前 Makefile 和程序支持的命令及**预期成功判据**。2026-10-02 实际执行的主机测试结果单独记录在 [VERIFICATION.md](VERIFICATION.md)；其余仿真、实时调度和板端步骤不标为已验证。已有二进制和源码中的历史注释不作为新测试证据。

## 环境与构建规则

推荐 Linux 或 WSL Ubuntu，准备 GCC、GNU Make、pthread 开发环境和标准 shell。代码使用 POSIX/Linux 的 socket、`poll`、`sched` 等接口；Makefile 注释中的 Windows-MinGW 不是本仓库已验证的原生 Windows 兼容承诺。

[根 Makefile](../Makefile) 将 `all`、`sim`、`test`、`verify`、`clean` 转发到 [gateway/Makefile](../gateway/Makefile)。所有实际输出位于 `gateway/build/`。

| 项目 | 当前规则 |
| --- | --- |
| 默认语言 / 优化 | `-std=c99 -O2 -g` |
| 告警 | `-Wall -Wextra -Werror -Wno-unused-parameter` |
| 头文件目录 | `-Iinclude -Iport`，相对于 `gateway/` |
| 核心源码 | `src/*.c` |
| 主机端口 | `port/port_host.c`、`port/port_hw_stub.c` |
| 测试 / 仿真入口 | `test/*.c` / `port/gw_host_sim.c` |
| 链接 | `-lpthread` |
| 产物 | `gateway/build/gw_tests`、`gateway/build/gw_host_sim` |

根入口显式指定 `CC=gcc`，避免 Make 的默认 `CC` 或外部环境造成命令含义不清。

## 快速开始

在仓库根目录运行：

```bash
make CC=gcc test
make CC=gcc sim
./gateway/build/gw_host_sim --link loopback --mode rtu --scenario normal --duration 2500
```

等价的子目录构建入口：

```bash
make -C gateway CC=gcc test
make -C gateway CC=gcc sim
```

- `make CC=gcc test` 构建测试程序并立即执行。总入口汇总用例组、断言总数和失败断言；退出 `0` 且显示 `PASS (0 失败)` 才可记录为本次通过。
- `make CC=gcc sim` 只编译仿真程序；默认 `make CC=gcc` 也是仿真构建，不会执行测试。
- `normal` 回环示例使用地址 1 的焊机和地址 2 的机器人模拟寄存器。预期能观察参数上报和汇总统计；数值来自程序初始化，不是实物测量。

Make 为增量构建，已有产物不等于新编译。需要强制重新编译且不清理旧文件时可以使用：

```bash
make -B CC=gcc test
make -B CC=gcc sim
```

不要把根 `build/` 的旧对象当作该规则的输出。`clean`/`rebuild` 目标会删除子目录产物，不是运行本指南的前置条件。

## 单元测试覆盖

测试范围由 [run_tests.c](../gateway/test/run_tests.c) 实际调用决定，而非头文件中是否存在测试函数声明。

| 源文件 | 源码中包含的主要断言 |
| --- | --- |
| [test_common.c](../gateway/test/test_common.c) | CRC16 标准向量、查表与逐位一致、分片累加、CRC 尾字节和大端读写 |
| [test_frame_sync.c](../gateway/test/test_frame_sync.c) | 分包、粘包、CRC 错、截断、噪声、重同步和超长帧 |
| [test_mem_pool.c](../gateway/test/test_mem_pool.c) | 池耗尽、重复/非法释放、并发分配归还；ring 回绕、满载、两段读取与直接写段接口 |
| [test_modbus_rtu.c](../gateway/test/test_modbus_rtu.c) | RTU 黄金帧、最大写数量、异常码、重试和寄存器往返；同文件还包含 TCP MBAP、半包/粘包、主从事务与写入测试 |
| [test_canopen.c](../gateway/test/test_canopen.c) | COB-ID、NMT 帧、心跳、对象字典、加速 SDO、分段辅助函数往返、PDO 和 EMCY |
| [test_bus_health.c](../gateway/test/test_bus_health.c) | 分类阈值、隔离冷却、好帧恢复、总线故障、错误窗口与日志 |
| [test_device_model.c](../gateway/test/test_device_model.c) | 设备注册、快照映射、心跳、离线缓存合并/满载；双槽保存、中断写入失败和出厂恢复 |
| [test_port.c](../gateway/test/test_port.c) | 消息队列、信号量、事件、递归锁、看门狗与主机栈观测 |
| [test_priority_inherit.c](../gateway/test/test_priority_inherit.c) | 优先级继承记账与对照、共享计数加锁循环；真实调度观察受运行模式约束 |

`test_modbus_rtu()` 内调用静态 `t_modbus_tcp()`，TCP 测试不是独立的 `test_modbus_tcp.c`。CANopen 的分段单帧往返测试不证明标准会话兼容；总线健康模块的恢复测试也不证明服务已形成总线 FAULT 恢复闭环。

优先级继承文件中“20 线程 x 2000 次”的用例标题与实际代码不一致：共享计数检查为单线程 20000 次加锁循环，不能将这一项写为多线程共享计数压力通过。队列和内存池文件另有真正的多线程测试。

这些测试不覆盖完整目标 BSP 编译、板端 DMA、满长 TCP ADU 入池安全、真实 Flash 擦写、跨进程保存或现场设备兼容性。通过主机测试后仍需单独验证这些风险。

## 整机仿真与日志

故障观察命令：

```bash
./gateway/build/gw_host_sim --link loopback --mode rtu --scenario mixed --duration 15000
```

`mixed` 源码中按每设备的请求次数推进：先正常应答，再对地址 1 注入 CRC 错，对地址 2 注入静默。`crc` 和 `offline` 也是可选场景，源码还处理 `short` 截断场景。它们不是固定时间点的现场故障注入，也不保证某一时长内得到固定恢复次数。

为了将文本参数行、统计行和诊断分别保存，可将生成文件放在已经忽略的构建目录：

```bash
./gateway/build/gw_host_sim --link loopback --mode rtu --scenario normal --duration 2500 \
  --stats-out gateway/build/stats.jsonl --report-out gateway/build/reports.jsonl \
  2>gateway/build/sim.log
```

输出行有 `GW_STATS_JSON ` 或 `GW_REPORT_JSON ` 前缀，整个文件不是无前缀的 JSON 文档；消费者应先区分记录类型，再解析后面的 JSON 对象。报告字段为 `dev`、`param`、`value`、`ts`，其中 `value` 是整数工程值，`ts` 来自主机单调毫秒计时，不是 Unix 时间戳。

`--polls <n>` 在主循环间歇检查是否达到尝试次数，不是精确限定事务总数；`--verbose` 增加诊断输出。仿真结束返回 `0` 仅说明主程序正常结束，它没有像测试程序一样对汇总统计做验收断言。

### TCP 与外部对端

`gw_host_sim` 的 `--link` 选择字节传输，`--mode` 选择应用封装；TCP socket 上仍可承载 RTU 字节，选择 socket 不自动等于 Modbus TCP。

内置 loopback 从站回调调用 `gw_mb_slave_process()`，只处理 RTU，不能用 `--link loopback --mode tcp` 宣称 TCP 仿真成功。TCP 模块的进程内主/从回环验证在单元测试中。需要外部 TCP 从站时，可使用以下**待验证的接入模板**：

```bash
./gateway/build/gw_host_sim --link tcp-connect --addr 127.0.0.1:15020 \
  --mode tcp --scenario normal --duration 2500 \
  --stats-out gateway/build/tcp-stats.jsonl --report-out gateway/build/tcp-reports.jsonl
```

此命令要求预先启动兼容的外部对端，仓库不附该服务；示例端口 `15020` 来自程序使用说明，不是标准端口配置。仿真会注册单元 1/2/3，外部对端需匹配对应示例寄存器窗口。`--scenario` 故障脚本只用于内置 loopback，不能控制外部从站。

stdio 模式会将协议请求写到 stdout，必须同时提供独立 `--stats-out` 和 `--report-out`，否则文本报告可能混入协议流。程序只强制检查统计文件参数，报告文件仍需调用者显式指定。

### 缺失的验证工具

`make verify` 调用 `python3 ../tools/verify_protocol.py --gateway-bin ...`，但当前仓库没有该文件。源码注释提到的 `tools/sim_slave.py` 同样缺失。因此不能列出“C/Python 端到端互操作已通过”，也不应把 `verify` 放入 CI 以掩盖缺失依赖。

## CI 与证据

[ci.yml](../.github/workflows/ci.yml) 对 `push`、`pull_request`、`workflow_dispatch` 触发，使用 Ubuntu runner、`actions/checkout@v4`、只读 `contents` 权限和 10 分钟 job 超时，执行：

```bash
make CC=gcc test
```

CI 不要求实时调度特权、不编译目标板、不执行仿真故障验收、不调用缺失的 Python 工具，也不发布构建产物。工作流文件存在不等于远端已运行或显示通过。

记录验证结果时应保留执行日期、源码版本、系统/GCC/Make 版本、准确命令、退出码、断言汇总和调度模式；[本次记录](VERIFICATION.md) 仅填写实际核验的信息，不补造未提供的版本和日志。主机默认 CFS，普通 CI 中的逻辑继承计数不能作为真实 RT-Thread 优先级调度时延证据。

## 目标板集成清单

这是前置条件清单，不是已测试的目标板快速开始：

1. 提供外部 RT-Thread BSP、内核、驱动、`SConstruct` 与工具链；引入 [gateway/SConscript](../gateway/SConscript)，避免同时链接两套端口或主机 `main`。
2. 在 BSP 应用中安排链路创建/打开、网关初始化、设备注册、sink 设置和启动；确认缺少公共声明的端口工厂与故障初始化接口如何接入。
3. 配置 UART/SAL/socket/CAN、NV、WDT、定时器和故障引脚，核对默认设备名与实际硬件，不能照抄占位引脚。
4. 验证非 `1000 Hz` 的 IPC 超时换算、API 返回语义、Flash 擦除单位和 CAN 帧字节封装。
5. 修正或约束满长 TCP ADU 入池路径，重新核算 `t_store` 的保存栈预算；验证统计/模型共享对象和停止时资源回收。
6. 再执行串口时序、断线、总线故障解除、队列满载、长时间压力、真实掉电和重启读回测试，并记录板型、配置和原始证据。

仓库没有硬件文件、具体板型或上述实测记录，不能用主机通过记录替代它们。
