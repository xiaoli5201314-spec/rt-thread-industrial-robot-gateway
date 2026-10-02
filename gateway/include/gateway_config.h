/*
 * gateway_config.h - 网关全局配置
 *
 * 这里是"系统级设计决策"的集中落点：线程优先级与栈深、队列深度、
 * 内存池容量、协议超时与重试策略。改这个文件就能改变整套实时性
 * 与资源占用特性，不需要翻遍各个 .c。
 *
 * 优先级约定：数值越小优先级越高（0 = 最高，31 = 最低，与 RT-Thread 一致）
 */
#ifndef GATEWAY_CONFIG_H
#define GATEWAY_CONFIG_H

#include "gw_types.h"

/* ================================================================== */
/* 1. 线程优先级与栈深（简历："逐线程校准栈深度"）                     */
/*                                                                     */
/* 栈深标注的是"标定值"：先用大栈跑满负载测高水位, 再取 1.6~2.0 倍     */
/* 余量定稿。gw_thread_stack_used() 会给出实测高水位, 详见 README      */
/* "栈深标定"表。                                                      */
/* ================================================================== */
#define GW_THREAD_PRIO_ACQ        8u    /* 采集：最高, 由中断信号量触发   */
#define GW_THREAD_PRIO_PARSE      12u   /* 解析：次高, 与采集背靠背       */
#define GW_THREAD_PRIO_REPORT     20u   /* 上报：网络/主机侧, 允许排队    */
#define GW_THREAD_PRIO_DIAG       24u   /* 诊断：周期性, 允许被抢占       */
#define GW_THREAD_PRIO_STORE      28u   /* 存储/文件：最低, 长耗时下沉    */

#define GW_THREAD_STACK_IRQ       1536u
#define GW_THREAD_STACK_ACQ       6144u
#define GW_THREAD_STACK_PARSE     5120u
#define GW_THREAD_STACK_REPORT    4096u
#define GW_THREAD_STACK_DIAG      4608u
#define GW_THREAD_STACK_STORE     3072u

#define GW_THREAD_NAME_ACQ        "t_acq"
#define GW_THREAD_NAME_PARSE      "t_parse"
#define GW_THREAD_NAME_REPORT     "t_report"
#define GW_THREAD_NAME_DIAG       "t_diag"
#define GW_THREAD_NAME_STORE      "t_store"

/* 看门狗监控位：每个线程一位, 全部喂到才认为系统健康 */
#define GW_WDT_BIT_IRQ            (1u << 0)
#define GW_WDT_BIT_ACQ            (1u << 1)
#define GW_WDT_BIT_PARSE          (1u << 2)
#define GW_WDT_BIT_REPORT         (1u << 3)
#define GW_WDT_BIT_DIAG           (1u << 4)
#define GW_WDT_BIT_STORE          (1u << 5)
#define GW_WDT_ALL_BITS           (GW_WDT_BIT_IRQ | GW_WDT_BIT_ACQ | \
                                   GW_WDT_BIT_PARSE | GW_WDT_BIT_REPORT | \
                                   GW_WDT_BIT_DIAG | GW_WDT_BIT_STORE)
#define GW_WDT_TIMEOUT_MS         2000u

/* 采集/中断/DMA 线程：比采集线程更高, 只做搬运不做解析 */
#define GW_THREAD_PRIO_IRQ        4u
#define GW_THREAD_STACK_IRQ       1536u
#define GW_THREAD_NAME_IRQ        "t_irq"

/* 事件标志组位定义 */
#define GW_EVT_START              (1u << 0)
#define GW_EVT_STOP               (1u << 1)
#define GW_EVT_PARAM_DIRTY        (1u << 2)
#define GW_EVT_DEV_RECOVERED      (1u << 3)
#define GW_EVT_POWER_FAIL         (1u << 4)

/* ================================================================== */
/* 2. 消息队列与缓冲区                                                 */
/* ================================================================== */
#define GW_RAW_RING_SIZE          2048u  /* DMA 环形缓冲, 2 的幂         */
#define GW_FRAME_QUEUE_DEPTH      48u    /* 采集 -> 解析 的帧队列        */
#define GW_REPORT_QUEUE_DEPTH     32u    /* 解析 -> 上报 的报文队列      */
#define GW_STORE_QUEUE_DEPTH      16u    /* 参数落盘队列                 */

#define GW_MAX_ADU_RTU            256u   /* Modbus RTU 最大 ADU          */
#define GW_MAX_ADU_TCP            260u   /* MBAP(7) + PDU(253)           */
#define GW_MODBUS_MAX_PDU         253u
#define GW_MODBUS_MAX_READ_REGS   125u   /* 功能码 03 单次最多读 125 个   */
#define GW_MODBUS_MAX_WRITE_REGS  123u   /* 功能码 16 单次最多写 123 个   */

/* ================================================================== */
/* 3. 内存池规划（简历："用内存池替代频繁动态分配"）                   */
/*                                                                     */
/* 三块定长池 + 一块字节池, 全部静态数组, 不含 malloc。理由：          */
/*   - 定长块池天然无外部碎片, 分配耗时恒定 O(1)；                     */
/*   - 帧对象、ADU、参数快照的尺寸是设计期已知的, 不需要通用堆。        */
/* ================================================================== */
#define GW_POOL_FRAME_BLOCK       64u    /* 帧描述符（含 8 字节 CAN 数据）*/
#define GW_POOL_FRAME_COUNT       96u
#define GW_POOL_ADU_BLOCK         256u   /* 协议 ADU 缓冲                */
#define GW_POOL_ADU_COUNT         24u
#define GW_POOL_PARAM_BLOCK       32u    /* 参数快照 / 缓存队列节点      */
#define GW_POOL_PARAM_COUNT       64u

/* ================================================================== */
/* 4. 协议超时与重试                                                   */
/* ================================================================== */
#define GW_MB_RESP_TIMEOUT_MS     200u   /* 从站响应超时                 */
#define GW_MB_RETRY_MAX           2u     /* 超时/CRC 错后的重试次数       */
#define GW_MB_INTER_FRAME_MS      2u     /* 主站两次请求之间的静默间隔    */

/* 帧同步 T3.5 静默间隔（微秒）。
 * Modbus 规程: 3.5 个字符时间。9600bps 下 1 字符 = 11bit/9600 ≈ 1146us,
 * T3.5 ≈ 4010us；115200bps 下 ≈ 334us。这里按 9600 档取 4000us, 并
 * 在 gateway_config.h 里集中定义, 便于按波特率裁剪。                  */
#define GW_T35_US                 4000u
#define GW_T15_US                 1700u  /* 字符间最大间隔（T1.5）      */

#define GW_FRAME_SYNC_BUF         320u   /* 单帧重组缓冲（>= 256 + 余量）*/

/* 现场可挂 32 个 Modbus 从站 + 16 个 CANopen 节点 */
#define GW_MAX_DEVICES            48u
#define GW_MAX_PARAM_MAP          256u   /* 在线参数映射条目上限         */
#define GW_PARAM_CACHE_DEPTH      32u    /* 掉线期间参数写缓存队列深度    */

/* ================================================================== */
/* 5. 可靠性与总线健康度                                               */
/* ================================================================== */
#define GW_CRC_ERR_ISOLATE_N      5u     /* 连续 N 次 CRC 错 -> 隔离设备 */
#define GW_TIMEOUT_ISOLATE_N      3u     /* 连续 N 次超时 -> 隔离设备    */
#define GW_RECOVER_GOOD_N         3u     /* 连续 N 帧正常 -> 重新纳入轮询*/
#define GW_ISOLATE_COOLDOWN_MS    3000u  /* 隔离后最小冷却时间           */
#define GW_BUS_ERR_WINDOW_MS      10000u /* 错误率统计窗口               */
#define GW_BUS_DEGRADE_RATIO_X100 1000u  /* 窗口内错误率 > 10% 则降级    */
#define GW_EVENT_LOG_DEPTH        64u    /* 总线事件日志环形深度         */

/* ================================================================== */
/* 6. 服务层行为                                                       */
/* ================================================================== */
#define GW_HEARTBEAT_TIMEOUT_MS   1000u  /* 设备心跳超时                 */
#define GW_HEARTBEAT_PERIOD_MS    250u   /* 网关侧心跳探测周期           */
#define GW_POLL_PERIOD_MS         20u    /* 采集线程基准轮询周期         */
#define GW_PARAM_SAVE_DEBOUNCE_MS 500u   /* 参数变更去抖, 避免频繁擦写    */
#define GW_CACHE_FLUSH_PERIOD_MS  100u   /* 离线缓存回放周期             */
#define GW_DEVICE_NAME_MAX        24u

/* ================================================================== */
/* 7. 编译期配置校验                                                   */
/* ================================================================== */
#if (GW_RAW_RING_SIZE & (GW_RAW_RING_SIZE - 1u)) != 0u
#error "GW_RAW_RING_SIZE must be a power of two (ring buffer uses mask wrap)"
#endif
#if GW_POOL_ADU_BLOCK < GW_MAX_ADU_RTU
#error "GW_POOL_ADU_BLOCK must be >= GW_MAX_ADU_RTU"
#endif
#if GW_FRAME_SYNC_BUF < GW_MAX_ADU_RTU
#error "GW_FRAME_SYNC_BUF must be >= GW_MAX_ADU_RTU"
#endif
#if GW_THREAD_PRIO_IRQ >= GW_THREAD_PRIO_ACQ || \
    GW_THREAD_PRIO_ACQ >= GW_THREAD_PRIO_PARSE || \
    GW_THREAD_PRIO_PARSE >= GW_THREAD_PRIO_REPORT || \
    GW_THREAD_PRIO_REPORT >= GW_THREAD_PRIO_DIAG || \
    GW_THREAD_PRIO_DIAG >= GW_THREAD_PRIO_STORE
#error "thread priority ordering must be IRQ < ACQ < PARSE < REPORT < DIAG < STORE"
#endif

#endif /* GATEWAY_CONFIG_H */
