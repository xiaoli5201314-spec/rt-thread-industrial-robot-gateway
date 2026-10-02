/*
 * diag.h - 诊断服务：看门狗 + 逐线程栈深标定 + 栈溢出检测
 *
 * 简历"看门狗复位保护"与"逐线程校准栈深度并开启栈溢出检测"的落地。
 *
 * 三个机制：
 *   1) 看门狗：每个被监控线程持有一个标志位, 全部门限内喂到才认为健康；
 *      超时则记录"哪几个线程没喂"并要求复位（复位前先尝试保存参数）。
 *   2) 栈深标定：周期采样每个线程的栈高水位, 取峰值。这个峰值就是
 *      README 里"栈深标定表"的来源 —— 栈深不是拍脑袋给的, 是量出来的。
 *   3) 栈溢出检测：端口层在栈底放置涂色红区, 每次采样校验红区完整性,
 *      一旦被写坏立即上报总线健康度并计数。
 */
#ifndef GW_DIAG_H
#define GW_DIAG_H

#include "gw_types.h"
#include "gateway_config.h"
#include "port_rtos.h"
#include "bus_health.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GW_DIAG_MAX_THREADS  8u

typedef struct {
    bool         used;
    const char  *name;
    uint32_t     wdt_bit;
    uint8_t      priority;          /* 逻辑优先级（0 最高）             */
    uint32_t     stack_size;        /* 设计栈深（字节）                 */
    gw_thread_t *handle;
    /* 运行数据 */
    uint32_t     feed_count;
    uint32_t     missed_count;
    int32_t      stack_used_peak;   /* 实测栈高水位峰值（字节）         */
    int32_t      stack_used_last;
    bool         overflow;          /* 曾经检测到栈溢出                 */
} gw_diag_thread_t;

typedef struct {
    gw_diag_thread_t threads[GW_DIAG_MAX_THREADS];
    uint32_t         count;
    uint32_t         wdt_timeout_ms;
    uint32_t         wdt_miss_count;
    uint32_t         wdt_check_count;
    uint32_t         reset_request_count;
    uint32_t         overflow_count;
    bool             reset_pending;
    uint32_t         last_missed_mask;
    gw_bus_health_t *bus;           /* 上报通道（可为 NULL）            */
} gw_diag_t;

int  gw_diag_init(gw_diag_t *dg, uint32_t wdt_timeout_ms);
void gw_diag_attach_bus(gw_diag_t *dg, gw_bus_health_t *bh);
int  gw_diag_register_thread(gw_diag_t *dg, gw_thread_t *handle, const char *name,
                             uint32_t wdt_bit, uint8_t priority, uint32_t stack_size);

/* 线程自己调用：喂自己的看门狗位 */
void gw_diag_feed(gw_diag_t *dg, uint32_t wdt_bit);

/* 诊断线程周期调用：采样栈水位 + 校验红区 + 检查看门狗
 * 返回未喂狗的位掩码（0 表示全部健康） */
uint32_t gw_diag_poll(gw_diag_t *dg, uint32_t now_ms);

/* 栈深标定：返回指定线程的实测峰值（字节）; 未找到返回 -1 */
int32_t gw_diag_stack_peak(gw_diag_t *dg, const char *name);
/* 打印标定表（诊断线程上电 3 秒后调用一次） */
void gw_diag_dump(gw_diag_t *dg);

#ifdef __cplusplus
}
#endif

#endif /* GW_DIAG_H */
