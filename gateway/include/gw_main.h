/*
 * gw_main.h - 服务层：四层架构的装配与线程编排
 *
 * 分层与数据流（详见 docs/ARCHITECTURE.md）：
 *
 *   ┌──────────────────────── 服务层 (Service) ────────────────────────┐
 *   │ t_acq 轮询调度 / t_report 上报 / t_diag 诊断 / t_store 存储       │
 *   └───────────────▲───────────────────────────┬─────────────────────┘
 *                   │ 消息队列 + 事件标志组      │ 内存池对象所有权转移
 *   ┌───────────────┴───────────────────────────▼─────────────────────┐
 *   │ 数据模型层 (Model)：设备注册表 / 心跳 / 参数在线映射 / 缓存队列    │
 *   │                    总线健康度分级 / 参数掉电存储                  │
 *   └───────────────▲───────────────────────────┬─────────────────────┘
 *                   │ PDU / ADU 结构             │ 工程值
 *   ┌───────────────┴───────────────────────────▼─────────────────────┐
 *   │ 协议适配层 (Adapter)：Modbus RTU / Modbus TCP / CANopen          │
 *   │                       帧同步状态机 / 超时重试                     │
 *   └───────────────▲───────────────────────────┬─────────────────────┘
 *                   │ 字节流                      │ 字节流
 *   ┌───────────────┴───────────────────────────▼─────────────────────┐
 *   │ 驱动层 (Driver)：DMA 环形缓冲 / 链路(Loopback, stdio, TCP, CAN)   │
 *   │                  t_irq 模拟 UART 接收中断 + DMA 搬运             │
 *   └─────────────────────────────────────────────────────────────────┘
 *
 * 关键实时性设计：
 *   - t_irq(prio 4) 只做"DMA 搬运 + 释放信号量", 不做任何解析；
 *     这样 115200bps 连续来帧时不会因为解析拖长中断/临界区。
 *   - t_acq(prio 8) 做协议收发与超时重试, 是唯一的总线主设备。
 *   - 解析(12) / 上报(20) / 诊断(24) / 存储(28) 依次降低优先级,
 *     把文件与网络这类长耗时操作下沉到最低优先级线程。
 */
#ifndef GW_MAIN_H
#define GW_MAIN_H

#include "gw_types.h"
#include "gateway_config.h"
#include "port_rtos.h"
#include "port_hw.h"
#include "ring_buffer.h"
#include "mem_pool.h"
#include "frame_sync.h"
#include "modbus_rtu.h"
#include "modbus_tcp.h"
#include "canopen.h"
#include "device_model.h"
#include "bus_health.h"
#include "param_store.h"
#include "diag.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 上报报文（解析线程 -> 上报线程） */
#define GW_REPORT_QUALITY_GOOD   0u
#define GW_REPORT_QUALITY_STALE  1u

typedef struct {
    uint8_t  dev_id;
    uint8_t  quality;
    uint16_t param_id;
    int32_t  value;
    uint32_t ts_ms;
} gw_report_t;

/* 原始帧（采集线程 -> 解析线程）；adu 指向内存池块, 所有权随之转移 */
typedef struct {
    uint8_t *adu;
    uint16_t len;
    uint8_t  dev_id;
    uint8_t  reserved;
    uint32_t ts_ms;
} gw_raw_frame_t;

/* 上报出口：目标板上换成 MQTT/以太网上报, PC 上写文件/stdout */
typedef int (*gw_report_sink_fn)(void *ctx, const gw_report_t *r);
/* 统计输出出口：默认走 printf；stdio 链路上 stdout 被协议字节占用,
 * 因此必须允许把统计行重定向到文件, 由宿主程序决定。 */
typedef void (*gw_stats_sink_fn)(void *ctx, const char *line);

typedef enum {
    GW_LINK_MODE_RTU = 0,     /* 链路上跑 Modbus RTU（CRC + 静默定界）*/
    GW_LINK_MODE_TCP          /* 链路上跑 Modbus TCP（MBAP 定界）    */
} gw_link_mode_t;

typedef struct {
    gw_link_mode_t mode;
    uint32_t       resp_timeout_ms;
    uint32_t       retries;
    uint32_t       poll_period_ms;
    bool           verbose;
} gw_gateway_cfg_t;

typedef struct gw_gateway {
    /* ---------------- 驱动层 ---------------- */
    gw_link_t    *link;
    gw_ringbuf_t  ring;
    uint8_t       ring_storage[GW_RAW_RING_SIZE];

    /* ---------------- 协议适配层 ---------------- */
    gw_mb_rtu_master_t rtu;
    gw_mb_tcp_master_t tcp;
    gw_frame_sync_t    fs;                 /* 环形缓冲 -> 完整帧        */
    uint8_t            fs_buf[GW_FRAME_SYNC_BUF];
    gw_link_mode_t     mode;
    gw_mb_tcp_asm_t    tcp_asm;            /* 环形缓冲 -> MBAP 定界     */
    uint8_t            pending[GW_MAX_ADU_TCP];
    uint32_t           pending_len;

    /* ---------------- 数据模型层 ---------------- */
    gw_devm_t        devm;
    gw_bus_health_t  bus;
    gw_param_store_t params;
    gw_param_blob_t  cfg_blob;

    /* ---------------- 服务层：同步对象 ---------------- */
    gw_diag_t    diag;
    gw_mq_t     *q_frames;      /* 采集 -> 解析                        */
    gw_mq_t     *q_reports;     /* 解析 -> 上报                        */
    gw_mq_t     *q_store;       /* 服务 -> 存储                        */
    gw_event_t  *evt;           /* 事件标志组                          */
    gw_sem_t    *sem_rx;        /* ISR/DMA -> 采集 的"数据到达"信号量   */
    gw_mutex_t  *lock_devm;     /* 设备表互斥（启用优先级继承）        */
    gw_mutex_t  *lock_bus;      /* 总线访问互斥（半双工 RS485 必须串行化）*/

    /* ---------------- 服务层：内存池 ---------------- */
    gw_mpool_t pool_frame;
    gw_mpool_t pool_adu;
    gw_mpool_t pool_param;
    uint8_t    pool_frame_mem[GW_POOL_FRAME_COUNT * GW_POOL_FRAME_BLOCK];
    uint8_t    pool_adu_mem[GW_POOL_ADU_COUNT * GW_POOL_ADU_BLOCK];
    uint8_t    pool_param_mem[GW_POOL_PARAM_COUNT * GW_POOL_PARAM_BLOCK];

    /* ---------------- 服务层：线程 ---------------- */
    gw_thread_t *t_irq;
    gw_thread_t *t_acq;
    gw_thread_t *t_parse;
    gw_thread_t *t_report;
    gw_thread_t *t_diag;
    gw_thread_t *t_store;

    gw_report_sink_fn report_sink;
    void             *report_sink_ctx;
    gw_stats_sink_fn  stats_sink;
    void             *stats_sink_ctx;

    volatile bool running;
    volatile bool stop_requested;

    /* ---------------- 统计（verify_protocol.py 读取） ---------------- */
    struct {
        uint64_t rx_bytes;
        uint64_t tx_frames;
        uint64_t frames_delivered;   /* 帧同步交付的完整帧            */
        uint64_t frames_crc_error;
        uint64_t frames_short;
        uint64_t frames_oversize;
        uint64_t resync_count;
        uint64_t reports;
        uint64_t report_drops;
        uint64_t frame_drops;        /* 队列满导致的丢帧              */
        uint64_t param_saves;
        uint64_t cache_flushed;
        uint64_t polls_attempted;
        uint64_t polls_ok;
        uint64_t polls_failed;
    } stats;

    uint32_t poll_cursor;            /* 轮询游标（公平调度）          */
    uint32_t isolate_events;
    uint32_t recover_events;
} gw_gateway_t;

/* 初始化：只做资源创建, 不启动线程 */
int  gw_gateway_init(gw_gateway_t *g, gw_link_t *link, const gw_gateway_cfg_t *cfg);
int  gw_gateway_register_device(gw_gateway_t *g, uint8_t addr, gw_protocol_t proto,
                                uint8_t profile, const char *name);
int  gw_gateway_start(gw_gateway_t *g);
void gw_gateway_stop(gw_gateway_t *g);
void gw_gateway_set_report_sink(gw_gateway_t *g, gw_report_sink_fn fn, void *ctx);
void gw_gateway_set_stats_sink(gw_gateway_t *g, gw_stats_sink_fn fn, void *ctx);
void gw_gateway_dump_stats(gw_gateway_t *g);

/* 供端口层中断/信号使用：唤醒采集线程（目标板上由 UART ISR 调用） */
void gw_gateway_notify_rx(gw_gateway_t *g);

/* 帧来源：把环形缓冲里的字节喂给帧同步状态机, 返回一帧完整数据 */
int  gw_gateway_frame_source(void *ctx, uint8_t *frame, uint32_t cap,
                             uint32_t *len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* GW_MAIN_H */
