# 协议子集、字段布局与兼容性

[返回 README](../README.md) | [架构文档](ARCHITECTURE.md) | [构建与测试](BUILD_AND_TEST.md)

本文描述**当前源码的编码与解析行为**，不是完整标准实现声明。偏移从 `0` 开始，长度以字节计；BE 表示高字节在前，LE 表示低字节在前。CAN 表格描述 CAN 数据字段，不含控制器层的物理帧位布局。

## 支持范围

| 协议 / 能力 | 当前实现 | 未提供 |
| --- | --- | --- |
| Modbus RTU / TCP | 共用寄存器 PDU，`0x03/0x04/0x06/0x10` | 线圈 `01/02/05/0F` 及其他功能码；完整标准一致性验收 |
| CANopen | 11 位 COB-ID、NMT 帧、心跳消费、对象字典、加速 SDO、PDO 解码、EMCY；分段辅助函数 | 完整 NMT 状态机、标准分段 SDO 会话、块传输、动态 PDO 协商、整机采集接入 |
| SYNC / TIME / LSS | COB-ID 分类 | 对应完整服务 |
| EtherNet/IP / DeviceNet | [gw_types.h](../gateway/include/gw_types.h) 的枚举及名称转换 | 协议栈、CIP 会话、设备互操作实现 |
| WebNet / MQTT | 无 WebNet；上报回调留有扩展位置 | HTTP/Web 管理、MQTT 客户端和连接管理 |

`modbus.h` 中定义了线圈功能码常量，但 [gw_mb_function_supported()](../gateway/src/modbus_pdu.c) 仅返回四个寄存器功能码为支持。常量存在不等于实现存在。

## 共用 Modbus PDU

实现入口：[modbus.h](../gateway/include/modbus.h)、[modbus_pdu.c](../gateway/src/modbus_pdu.c)。

| 请求 | 字段顺序 | PDU 长度 | 数量约束 |
| --- | --- | --- | --- |
| `03` 读保持 / `04` 读输入 | `FC[0]`、起始地址 BE16 `[1..2]`、数量 BE16 `[3..4]` | 5 | `1..125` |
| `06` 写单寄存器 | `FC[0]`、地址 BE16 `[1..2]`、值 BE16 `[3..4]` | 5 | 单个 16 位值 |
| `10` 写多寄存器 | `FC[0]`、起始地址 BE16 `[1..2]`、数量 BE16 `[3..4]`、字节数 `[5]`、BE16 数据 `[6..]` | `6 + 2N` | `1..123`，字节数等于 `2N` |

| 响应 | 字段顺序 | PDU 长度 |
| --- | --- | --- |
| `03/04` | `FC[0]`、字节数 `[1]`、BE16 数据 `[2..]` | `2 + 2N` |
| `06` | 回显请求的地址与值 | 5 |
| `10` | `FC[0]`、起始地址 BE16 `[1..2]`、数量 BE16 `[3..4]` | 5 |
| 异常 | `FC \| 0x80` `[0]`、异常码 `[1]` | 2 |

核心函数：`gw_mb_pdu_build_read()`、`gw_mb_pdu_build_write_single()`、`gw_mb_pdu_build_write_multi()`、`gw_mb_pdu_parse_request()`、`gw_mb_pdu_parse_response()`；响应由 `gw_mb_pdu_build_read_response()`、`gw_mb_pdu_build_write_response()` 和 `gw_mb_pdu_build_exception()` 构造。

从站路径可对不支持功能、地址越界和非法数量生成 `01/02/03` 异常码。主站将异常响应与 CRC/超时区分，服务不会因为单纯异常码触发设备隔离；不应将异常应答称为物理总线损坏。

## Modbus RTU

入口：[modbus_rtu.c](../gateway/src/modbus_rtu.c)、[gw_crc.c](../gateway/src/gw_crc.c)、[frame_sync.c](../gateway/src/frame_sync.c)。

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| `0` | 1 | 从站地址 |
| `1` | PDU 长度 | PDU，首字节为功能码 |
| `总长 - 2` | 1 | CRC16 低字节 |
| `总长 - 1` | 1 | CRC16 高字节 |

RTU ADU 总长为 `PDU + 3`，上限配置为 256。读请求、单写请求/响应、多写响应均为 8 字节；读响应 `5 + 2N`；多写请求 `9 + 2N`；异常响应 5 字节。寄存器大端与 CRC 尾部小端是两种不同规则。

CRC16 初值 `0xFFFF`，逐位实现使用反射多项式 `0xA001`；`gw_crc16_modbus()`、`gw_crc16_modbus_fast()` 和 `gw_crc16_modbus_update()` 分别用于逐位、查表和分片累加，追加/校验接口见同文件。

[test_modbus_rtu.c](../gateway/test/test_modbus_rtu.c) 内的请求黄金帧示例：

```text
01 03 00 00 00 0A C5 CD
```

这是地址 1、功能 `03`、起始 `0x0000`、读取 10 个寄存器，尾部为低字节先行的 CRC；它是代码测试向量，不是现场抓包。

`gw_mb_rtu_build_read()`、`gw_mb_rtu_build_write_single()`、`gw_mb_rtu_build_write_multi()` 构造 ADU，`gw_mb_rtu_master_txn()` 处理事务，`gw_mb_slave_process()` 是内置 RTU 从站处理入口。默认响应超时 200 ms，重试 2 次，即最多 3 次尝试；RTU 重试间隔配置 2 ms。

帧同步状态为 IDLE、RECEIVING、RESYNC，利用 `gw_mb_rtu_expected_len()`、CRC 候选帧检查和滑动重同步识别噪声、粘包与残帧。`GW_T35_US=4000` 用于静默结算，`GW_T15_US=1700` 虽有定义，但没有完整的逐字符 T1.5 约束实现。UART 默认是端口中的 `9600/8N1`，不能仅凭配置注释就宣称所有波特率的标准时序已覆盖。

## Modbus TCP

入口：[modbus_tcp.h](../gateway/include/modbus_tcp.h)、[modbus_tcp.c](../gateway/src/modbus_tcp.c)。

| 偏移 | 长度 | 字段 / 编码 |
| --- | --- | --- |
| `0..1` | 2 | Transaction ID，BE16 |
| `2..3` | 2 | Protocol ID，BE16；构造器写 `0` |
| `4..5` | 2 | Length，BE16；值为 `1 + PDU 长度` |
| `6` | 1 | Unit ID |
| `7..` | PDU 长度 | 共用 PDU |

MBAP 共 7 字节，ADU 总长 `6 + Length = 7 + PDU`，配置上限 260；不追加 RTU CRC。Length 包含 Unit ID，但不包含前 6 字节。

- `gw_mb_tcp_build()` 限制 PDU 为 `1..253` 并生成 MBAP；`gw_mb_tcp_parse()` 检查非零 PID、最小 Length 和总长一致性。独立 parse 函数没有完整的最大长度限制，不能当作通用安全输入验证器。
- `gw_mb_tcp_asm_feed()` 累积半包，按 MBAP Length 多次回调输出粘连帧；PID 或长度异常时移除最旧 1 字节重同步。其缓冲容量约束与 PDU 功能码合法性是不同层次的检查。
- `gw_mb_tcp_master_txn()` 从事务号 1 开始递增并跳过 0，同一事务重试使用同一 TID；响应匹配包含 TID 和 Unit ID 检查。
- `gw_mb_tcp_slave_process()` 将 TCP 请求交给共用 PDU/寄存器从站逻辑。[测试](../gateway/test/test_modbus_rtu.c) 中有独立 TCP loopback 回调，主机仿真内置从站却仍是 RTU，二者不要混淆。

**服务层容量限制：** 允许构造的最大 TCP ADU 为 260 字节，服务 ADU 池块只有 256 字节，而且响应复制没有池块容量判断。最大读响应为 `7 + 2 + 2 * 125 = 259` 字节，同样超过池块。默认示例仅读较小窗口，不证明满长事务在整机中安全。

## 示例设备映射

以下内容直接来自 [device_model.c](../gateway/src/device_model.c)，是**示例档案**，不是厂商寄存器手册。地址为零基原始偏移，不使用 `40001` 等人类编号；32 位值占两个寄存器且高字在前。工程值使用整数 `raw * num / den`，会截断小数；档案中的单位字符串并非额外的定点序列化规则。

| 档案 | 协议标记 | 保持寄存器轮询窗口 | 额外表 |
| --- | --- | --- | --- |
| `digital-welder` | RTU | `0x0000` 起，13 个 | 输入表 `0x0000` 起，3 个 |
| `six-axis-robot` | RTU | `0x0000` 起，18 个 | 无 |
| `plc-remote-io` | TCP | `0x0000` 起，8 个 | 无 |
| `canopen-io-node` | CANopen | 无 Modbus 窗口 | OD 示例 |

协议标记不意味着每台设备独立选择物理链路；整机仍按全局模式运行。目前采集仅请求保持寄存器，输入表未接入第二次读取，CANopen 档案也未接入轮询。

| 档案 | 寄存器 / OD 索引:子索引 | 参数 ID | 字段 | 类型 | 比例 `num/den` |
| --- | --- | --- | --- | --- | --- |
| 焊机保持 | `0x0000` | `0x1001` | `welding_current_set` | U16 | `1/10` |
| 焊机保持 | `0x0001` | `0x1002` | `welding_voltage_set` | U16 | `1/10` |
| 焊机保持 | `0x0002` | `0x1003` | `wire_feed_speed` | U16 | `1/100` |
| 焊机保持 | `0x0003` | `0x1004` | `welding_current_act` | U16 | `1/10` |
| 焊机保持 | `0x0004` | `0x1005` | `welding_voltage_act` | U16 | `1/10` |
| 焊机保持 | `0x0005` | `0x1006` | `weld_state` | U16 | `1/1` |
| 焊机保持 | `0x0006` | `0x1007` | `fault_code` | U16 | `1/1` |
| 焊机保持 | `0x0007..0008` | `0x1008` | `weld_time_total` | U32 | `1/1` |
| 焊机保持 | `0x0009` | `0x1009` | `gas_flow` | U16 | `1/10` |
| 焊机保持 | `0x000A` | `0x100A` | `welder_enable` | U16 | `1/1` |
| 焊机保持 | `0x000B` | `0x100B` | `process_no` | U16 | `1/1` |
| 焊机保持 | `0x000C` | `0x100C` | `arc_force` | S16 | `1/10` |
| 焊机输入 | `0x0000` | `0x1101` | `dc_bus_voltage` | U16 | `1/10` |
| 焊机输入 | `0x0001` | `0x1102` | `igbt_temp` | U16 | `1/1` |
| 焊机输入 | `0x0002` | `0x1103` | `wire_motor_current` | U16 | `1/100` |
| 机器人 | `0x0000` | `0x2001` | `run_mode` | U16 | `1/1` |
| 机器人 | `0x0001` | `0x2002` | `program_no` | U16 | `1/1` |
| 机器人 | `0x0002..0007` | `0x2003..2008` | `joint1_angle` 到 `joint6_angle`，逐项对应 | S16 | `1/100` |
| 机器人 | `0x0008..0009` | `0x2009` | `tcp_x` | S32 | `1/100` |
| 机器人 | `0x000A..000B` | `0x200A` | `tcp_y` | S32 | `1/100` |
| 机器人 | `0x000C..000D` | `0x200B` | `tcp_z` | S32 | `1/100` |
| 机器人 | `0x000E` | `0x200C` | `alarm_code` | U16 | `1/1` |
| 机器人 | `0x000F..0010` | `0x200D` | `cycle_count` | U32 | `1/1` |
| 机器人 | `0x0011` | `0x200E` | `servo_enable` | U16 | `1/1` |
| PLC | `0x0000` | `0x3001` | `line_cycle_time` | U16 | `1/10` |
| PLC | `0x0001..0002` | `0x3002` | `workpiece_count` | U32 | `1/1` |
| PLC | `0x0003` | `0x3003` | `station_status` | BITFIELD | `1/1` |
| PLC | `0x0004` | `0x3004` | `estop_state` | U16 | `1/1` |
| PLC | `0x0005` | `0x3005` | `safety_door` | U16 | `1/1` |
| PLC | `0x0006` | `0x3006` | `air_pressure` | U16 | `1/10` |
| PLC | `0x0007` | `0x3007` | `ambient_temp` | S16 | `1/10` |
| CANopen IO | `0x6000:1..4` | `0x4001..4004` | `do_ch1` 到 `do_ch4`，逐项对应 | U8 | `1/1` |
| CANopen IO | `0x6401:1` | `0x4011` | `di_ch1_8` | U16 | `1/1` |
| CANopen IO | `0x6401:2` | `0x4012` | `di_ch9_16` | U16 | `1/1` |

`gw_devm_field_by_param()` 还会查共享的输入参数表；快照没有独立功能码/寄存器空间标签。不能仅凭同一个原始地址就假设该服务已区分保持和输入快照。`gw_devm_read_param()` 返回 `int32_t`，U32 档案也不构成任意无符号 32 位范围的完整接口承诺。

## CANopen 子集

入口：[canopen.h](../gateway/include/canopen.h)、[canopen.c](../gateway/src/canopen.c)。`gw_can_frame_t` 包含 ID、DLC、`data[8]` 等字段；`gw_canopen_decode_cob()` 仅接受数值不超过 `0x7FF` 的 ID，当前不支持 29 位扩展 ID 或 CAN FD。分发器检查 DLC 不超过 8，但结构中的扩展标记本身不是完整的帧合法性校验。

### COB-ID 分类

| 类型 | 当前识别基址 |
| --- | --- |
| NMT | `0x000` |
| SYNC / EMCY | `0x080` / `0x080 + node` |
| TIME | `0x100`；分类器还将该 `0x780` 掩码区间归为 TIME |
| TPDO 1..4 | `0x180 / 0x280 / 0x380 / 0x480 + node` |
| RPDO 1..4 | `0x200 / 0x300 / 0x400 / 0x500 + node` |
| SDO 响应 / 请求 | `0x580 + node` / `0x600 + node` |
| 心跳 | `0x700 + node` |
| LSS TX / RX | `0x7E4 / 0x7E5`，只分类 |

对大部分连接使用 `base = id & 0x780`、`node = id & 0x7F`。这是分类逻辑，不是对全部预定义连接和节点合法性的严格标准校验。

### NMT 与心跳

`gw_canopen_nmt_build()` 生成 ID `0x000`、DLC 2，`data[0]` 为命令，`data[1]` 为节点号，0 表示广播；节点参数不得超过 `0x7F`。定义命令 `01` 启动、`02` 停止、`80` 预操作、`81` 重置节点、`82` 重置通信。函数不会实施完整的 NMT 状态迁移，也没有完整命令合法性过滤。

心跳数据首字节的状态常量为 `00` Boot-up、`04` Stopped、`05` Operational、`7F` Pre-operational。`gw_canopen_hb_register()`、`gw_canopen_hb_feed()`、`gw_canopen_hb_check()` 管理最多 32 个节点；超时参数为 0 时取 1000 ms。消费器存储收到的状态，不据此驱动完整 NMT 会话；当前 feed 路径也可对零 DLC 帧更新时间，不能代替严格心跳格式检查。

### SDO 加速传输

SDO 构造器使用 DLC 8，字段布局：

| 数据偏移 | 长度 | 内容 |
| --- | --- | --- |
| `0` | 1 | Command Specifier，CS |
| `1..2` | 2 | 对象索引 LE16 |
| `3` | 1 | 子索引 |
| `4..7` | 4 | 值或中止码，LE32；不足长度的构造请求高位补 0 |

| 有效长度 | 下载请求 CS | 上传响应 CS |
| --- | --- | --- |
| 1 字节 | `0x2F` | `0x4F` |
| 2 字节 | `0x2B` | `0x4B` |
| 3 字节 | `0x27` | `0x47` |
| 4 字节 | `0x23` | `0x43` |

`gw_sdo_build_download_expedited()` 构造请求，`gw_sdo_build_upload_request()` 使用 CS `0x40`；`gw_sdo_parse()` 识别加速请求/响应、`0x20` 下载确认、`0x60` 辅助类型与 `0x80` 中止。中止码从 `data[4..7]` 取 LE32 并返回 `GW_ERR_EXCEPTION`。解析函数没有完整节点/COB-ID 会话匹配，且返回值组装会读取全部四个值字节，调用方需按有效长度解释。

### 分段辅助函数，非完整标准会话

- `gw_sdo_build_initiate_download()` 使用 CS `0x21`，索引/子索引同上，`data[4..7]` 为总长度 LE32。
- `gw_sdo_build_segment()` 支持 `len=1..7`，当前实际写 `data[0] = (toggle ? 0x10 : 0) | (7 - len)`，`data[1..len]` 为数据，DLC 为 8。
- `gw_sdo_parse_segment()` 用 bit 4 取 toggle、低 4 位取 unused，输出长度 `7 - unused`；`gw_sdo_build_segment_ack()` 生成 `0x20 | toggle_bit`。

这些是当前实现的位布局，**不能当作标准兼容的分段 SDO 编码**：没有独立 last-segment 参数，也没有完整会话、toggle 连续性校验、总长度控制、超时、重试或块传输。单元测试只做这些辅助函数的构造/解析往返，不证明可与工业 CANopen 节点互操作。

### PDO 与 EMCY

`gw_pdo_decode_to_od()` 根据映射表顺序消费数据，只支持 8/16/32 位、整字节条目，查找 OD 后通过 `gw_od_write_u32()` 写入。映射结构容量为 8 条；没有动态映射协商或 PDO 生产者。

**当前 PDO 实现把多字节字段按大端组装**，不同于本文件 SDO 的小端规则。不能把源码中的“大端”注释当作 CANopen 通用标准结论；部署前必须按实际节点的字节序修正或验证，不应宣称标准 PDO 互操作已经完成。

EMCY 数据为 `data[0..1]` 错误码 LE16、`data[2]` 错误寄存器、`data[3..7]` 五字节厂商数据。`gw_canopen_emcy_parse()` 当前接受最低 DLC 2，未提供的其余字段补零，不是严格要求 8 字节的完整检查器。

## 参数槽布局

参数保存不是工业总线协议，但其字节编码由 [param_store.c](../gateway/src/param_store.c) 明确定义，可用于审查 NV 镜像。

两个槽位各 2048 字节，起始偏移为 `0` 和 `2048`；NV 总容量 4096 字节。最大 payload 是宏 `2048 - 32 = 2016`，不能直接用槽长减头长推导出 2032。

| 槽内偏移 | 长度 | 字段与编码 |
| --- | --- | --- |
| `0..3` | 4 | magic BE32，`0x47575031`，即 `GWP1` |
| `4..7` | 4 | seq BE32 |
| `8..9` | 2 | payload_len BE16 |
| `10..11` | 2 | header CRC16，低字节先；实际覆盖前 **10** 字节 |
| `12..13` | 2 | payload CRC16，BE16；覆盖 payload |
| `14..15` | 2 | reserved BE16，当前保存为 0 |
| `16..` | payload_len | 原样复制的载荷 |

槽头注释称 CRC 覆盖前 12 字节，但 `hdr_encode()`/`hdr_decode()` 实际传入长度 `10`，此表以代码为准。保存先将整页填 `0xFF`，写非当前槽，整页回读一致后才更新活动槽。

服务传入的载荷是 `gw_param_blob_t` 的原始内存，没有逐字段编码；其 `magic`、`version`、设备数、轮询/心跳配置及焊机/机器人默认值等不能被描述为跨编译器固定偏移的网络格式。槽头序号比较采用无符号差值；载入先挑最新有效头再校验载荷，最新载荷坏时不会再次回退旧槽。

主机 NV 仅进程内静态数组。双槽软件测试不等于跨进程持久化、真实 Flash 断电原子性或目标板寿命验证。
