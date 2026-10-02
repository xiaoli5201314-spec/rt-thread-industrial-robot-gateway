/*
 * gw_main.c - 服务层：四层装配、线程编排、同步对象用法
 *
 * 线程分工与同步关系（README "线程优先级表" 的数据来源）：
 *
 *   t_irq    prio 4   栈 1536  DMA/中断搬运：link -> 环形缓冲 -> sem_rx
 *   t_acq    prio 8   栈 2048  总线主设备：轮询调度、超时重试、发帧入队
 *   t_parse  prio 12  栈 4096  帧语义解析、数据模型更新、生成上报报文
 *   t_report prio 20  栈 4608  上报出口（网络/文件），批量写
 *   t_diag   prio 24  栈 3072  看门狗、栈水位、心跳超时、总线健康度
 *   t_store  prio 28  栈 4096  参数掉电保存、离线参数缓存回放
 *
 * 同步对象：
 *   sem_rx      : ISR/DMA -> 采集线程 的"数据到达"信号量（计数型）
 *   q_frames    : 采集 -> 解析   消息队列（传内存池指针, 所有权转移）
 *   q_reports   : 解析 -> 上报   消息队列（定长结构体）
 *   q_store     : 服务 -> 存储   消息队列（参数落盘请求）
 *   evt         : 事件标志组（START/STOP/PARAM_DIRTY/DEV_RECOVERED/POWER_FAIL）
 *   lock_bus    : 半双工总线互斥（RT-Thread RT_IPC_FLAG_PRIO 等价物）
 *   lock_devm   : 设备表互斥（启用优先级继承, 防优先级反转）
 *   pool_adu    : ADU 缓冲内存池（替代 malloc, 避免堆碎片）
 *   pool_frame  : 帧描述符内存池
 *   pool_param  : 参数快照内存池
 */
#include "gw_main.h"
#include "gw_log.h"
#include "gw_crc.h"

#include <stdio.h>
#include <string.h>

/* ================================================================== */
/* 内部工具                                                            */
/* ================================================================== */
static void gw_gateway_fs_cb(void *ctx, const gw_fs_event_t *ev)
{
    gw_gateway_t *g = (gw_gateway_t *)ctx;

    switch (ev->kind) {
    case GW_FS_EV_FRAME:
        g->stats.frames_delivered++;
        if (ev->len <= sizeof(g->pending)) {
            memcpy(g->pending, ev->data, ev->len);
            g->pending_len = ev->len;
        }
        break;
    case GW_FS_EV_CRC_ERROR:   g->stats.frames_crc_error++; break;
    case GW_FS_EV_SHORT_FRAME: g->stats.frames_short++;     break;
    case GW_FS_EV_OVERSIZE:    g->stats.frames_oversize++;  break;
    case GW_FS_EV_RESYNC:      g->stats.resync_count++;     break;
    default: break;
    }
}

static void gw_tcp_frame_cb(void *ctx, const uint8_t *adu, uint32_t len)
{
    gw_gateway_t *g = (gw_gateway_t *)ctx;

    g->stats.frames_delivered++;
    if (len <= sizeof(g->pending)) {
        memcpy(g->pending, adu, len);
        g->pending_len = len;
    }
}

/* 把环形缓冲里的字节搬给定界逻辑（RTU 用帧同步, TCP 用 MBAP 长度） */
static void gw_gateway_drain_ring(gw_gateway_t *g)
{
    while (gw_rb_used(&g->ring) > 0u) {
        const uint8_t *s1 = NULL;
        const uint8_t *s2 = NULL;
        uint32_t       l1 = 0u;
        uint32_t       l2 = 0u;

        if (gw_rb_contiguous(&g->ring, &s1, &l1, &s2, &l2) != GW_OK) {
            return;
        }
        if ((l1 == 0u) && (l2 == 0u)) {
            return;
        }

        if (g->mode == GW_LINK_MODE_RTU) {
            if (l1 > 0u) {
                gw_frame_sync_feed(&g->fs, s1, l1, gw_port_now_us());
            }
            if (l2 > 0u) {
                gw_frame_sync_feed(&g->fs, s2, l2, gw_port_now_us());
            }
        } else {
            if (l1 > 0u) {
                gw_mb_tcp_asm_feed(&g->tcp_asm, s1, l1, gw_tcp_frame_cb, g);
            }
            if (l2 > 0u) {
                gw_mb_tcp_asm_feed(&g->tcp_asm, s2, l2, gw_tcp_frame_cb, g);
            }
        }

        (void)gw_rb_skip(&g->ring, l1 + l2);
        if (g->pending_len > 0u) {
            return;      /* 已经拿到一帧, 先交给上层 */
        }
    }
}

void gw_gateway_notify_rx(gw_gateway_t *g)
{
    if (g != NULL) {
        (void)gw_sem_release(g->sem_rx);
    }
}

/* 帧来源：协议适配层从此处取帧, 不再直接碰链路 */
int gw_gateway_frame_source(void *ctx, uint8_t *frame, uint32_t cap,
                            uint32_t *len, uint32_t timeout_ms)
{
    gw_gateway_t *g = (gw_gateway_t *)ctx;
    uint32_t      deadline;

    if ((g == NULL) || (frame == NULL) || (len == NULL)) {
        return GW_ERR_PARAM;
    }

    /* 静默间隔兜底：先结算可能已经结束的帧 */
    if (g->mode == GW_LINK_MODE_RTU) {
        gw_frame_sync_tick(&g->fs, gw_port_now_us());
    }
    if (g->pending_len > 0u) {
        goto deliver;
    }

    deadline = gw_port_tick_ms() + timeout_ms;
    for (;;) {
        int32_t remain;

        gw_gateway_drain_ring(g);
        if (g->pending_len > 0u) {
            break;
        }
        if (g->mode == GW_LINK_MODE_RTU) {
            gw_frame_sync_tick(&g->fs, gw_port_now_us());
            if (g->pending_len > 0u) {
                break;
            }
        }

        remain = (int32_t)(deadline - gw_port_tick_ms());
        if (remain <= 0) {
            return GW_ERR_TIMEOUT;
        }
        (void)gw_sem_take(g->sem_rx, remain);
    }

deliver:
    if (g->pending_len > cap) {
        g->pending_len = 0u;
        return GW_ERR_OVERFLOW;
    }
    memcpy(frame, g->pending, g->pending_len);
    *len = g->pending_len;
    g->pending_len = 0u;
    return GW_OK;
}

/* 从原始 ADU 里取出寄存器值（两种链路模式统一出口） */
static int gw_gateway_extract_values(gw_gateway_t *g, const uint8_t *adu,
                                     uint32_t len, uint16_t *values,
                                     uint16_t *count)
{
    if (g->mode == GW_LINK_MODE_RTU) {
        gw_mb_response_t r;
        int rc = gw_mb_rtu_parse_response(adu, len, &r);
        if (rc != GW_OK) {
            return rc;
        }
        if (r.value_count > *count) {
            return GW_ERR_OVERFLOW;
        }
        memcpy(values, r.values, (size_t)r.value_count * sizeof(uint16_t));
        *count = r.value_count;
        return GW_OK;
    }

    {
        gw_mbap_t       hdr;
        const uint8_t  *pdu = NULL;
        uint32_t        pdu_len = 0u;
        gw_mb_response_t r;
        int rc = gw_mb_tcp_parse(adu, len, &hdr, &pdu, &pdu_len);
        if (rc != GW_OK) {
            return rc;
        }
        rc = gw_mb_pdu_parse_response(pdu, pdu_len, &r);
        if (rc != GW_OK) {
            return rc;
        }
        if (r.value_count > *count) {
            return GW_ERR_OVERFLOW;
        }
        memcpy(values, r.values, (size_t)r.value_count * sizeof(uint16_t));
        *count = r.value_count;
        return GW_OK;
    }
}

/* ================================================================== */
/* 线程 1：t_irq —— DMA / 接收中断搬运                                 */
/*                                                                     */
/* 目标板上这个线程并不存在, 其职责由 UART IDLE 中断 + DMA 完成中断    */
/* 承担：中断里只做 gw_rb_dma_commit() 和 gw_sem_release()。PC 仿真    */
/* 下用一个最高优先级线程轮流扮演"中断", 从而让环形缓冲与信号量这条    */
/* 路径被真实执行, 而不是靠打桩跳过。                                  */
/* ================================================================== */
static void gw_thread_irq(void *arg)
{
    gw_gateway_t *g = (gw_gateway_t *)arg;

    while (g->running) {
        uint32_t maxlen = 0u;
        uint8_t *p = gw_rb_dma_write_ptr(&g->ring, &maxlen);

        if ((p != NULL) && (maxlen > 0u)) {
            int n = gw_link_recv(g->link, p, maxlen, 10);
            if (n > 0) {
                gw_rb_dma_commit(&g->ring, (uint32_t)n);
                g->stats.rx_bytes += (uint64_t)n;
                gw_gateway_notify_rx(g);       /* == ISR 释放信号量 */
            } else if (n < 0 && n != GW_ERR_TIMEOUT) {
                /* 链路断开：不退出线程, 交给采集线程的超时与总线健康度处理 */
            }
        } else {
            gw_port_delay_ms(1);               /* 环形缓冲满, 等消费侧腾空间 */
        }
        gw_diag_feed(&g->diag, GW_WDT_BIT_IRQ);
    }
}

/* ================================================================== */
/* 线程 2：t_acq —— 轮询调度与超时重试                                 */
/* ================================================================== */
static void gw_acq_poll_device(gw_gateway_t *g, gw_device_t *d)
{
    uint8_t  req[GW_MAX_ADU_TCP];
    uint8_t  resp[GW_MAX_ADU_TCP];
    uint32_t req_len = 0u;
    uint32_t resp_len = 0u;
    uint64_t crc_before;
    uint64_t short_before;
    uint32_t now = gw_port_tick_ms();
    int      rc;

    /* 半双工总线上"发-收"必须整体串行, 用互斥量保护（带优先级继承） */
    if (gw_mutex_take(g->lock_bus, GW_WAIT_FOREVER) != GW_OK) {
        return;
    }

    crc_before   = g->stats.frames_crc_error;
    short_before = g->stats.frames_short;
    g->stats.polls_attempted++;

    if (g->mode == GW_LINK_MODE_RTU) {
        rc = gw_mb_rtu_build_read(d->addr, GW_MB_FC_READ_HOLDING, d->reg_base,
                                  d->reg_count, req, sizeof(req), &req_len);
        if (rc == GW_OK) {
            g->stats.tx_frames++;
            rc = gw_mb_rtu_master_txn(&g->rtu, req, req_len, resp, sizeof(resp),
                                      &resp_len);
        }
    } else {
        uint8_t  pdu[8];
        uint32_t pdu_len = 0u;
        rc = gw_mb_pdu_build_read(GW_MB_FC_READ_HOLDING, d->reg_base, d->reg_count,
                                  pdu, sizeof(pdu), &pdu_len);
        if (rc == GW_OK) {
            g->stats.tx_frames++;
            rc = gw_mb_tcp_master_txn(&g->tcp, d->addr, pdu, pdu_len, resp,
                                      sizeof(resp), &resp_len);
        }
    }

    if (rc == GW_OK) {
        /* 关键设计：采集线程**不做语义解析**, 只把原始 ADU 通过内存池
         * 交给解析线程。这样高优先级线程的执行时间与报文复杂度无关,
         * 抖动可控。 */
        uint8_t *blk = (uint8_t *)gw_mpool_alloc(&g->pool_adu);
        if (blk != NULL) {
            gw_raw_frame_t msg;
            memcpy(blk, resp, resp_len);
            msg.adu      = blk;
            msg.len      = (uint16_t)resp_len;
            msg.dev_id   = d->id;
            msg.reserved = 0u;
            msg.ts_ms    = now;
            if (gw_mq_send(g->q_frames, &msg, GW_NO_WAIT) != GW_OK) {
                g->stats.frame_drops++;
                (void)gw_mpool_free(&g->pool_adu, blk);
            }
        } else {
            g->stats.frame_drops++;            /* 内存池耗尽：可观测的丢帧 */
        }
        g->stats.polls_ok++;
    } else {
        gw_err_class_t cls;
        uint32_t       attempts;
        uint32_t       k;

        if (rc == GW_ERR_EXCEPTION) {
            cls = GW_ERRCLASS_EXCEPTION;
        } else if ((g->stats.frames_crc_error > crc_before) ||
                   (g->stats.frames_short > short_before)) {
            cls = GW_ERRCLASS_CRC;
        } else if ((rc == GW_ERR_TIMEOUT) || (rc == GW_ERR_IO)) {
            cls = GW_ERRCLASS_TIMEOUT;
        } else {
            cls = GW_ERRCLASS_ILLEGAL_FRAME;
        }

        /*
         * 按"失败尝试次数"而不是"失败事务次数"上报总线健康度。
         * 这一点很关键：一次事务内部重试了 3 次才失败, 说明总线上
         * 真的错了 3 次；只算 1 次会显著低估错误率, 导致隔离阈值
         * 永远达不到 —— 现场表现就是"设备一直在错, 但永远不被隔离"。
         */
        attempts = (g->mode == GW_LINK_MODE_RTU) ? g->rtu.last_attempts
                                                 : g->tcp.last_attempts;
        if (attempts == 0u) {
            attempts = 1u;
        }
        for (k = 0u; k < attempts; k++) {
            (void)gw_bus_health_report(&g->bus, d->id, cls, now);
        }

        g->stats.polls_failed++;
        gw_devm_report_poll_fail(&g->devm, d, cls, now);
    }

    (void)gw_mutex_release(g->lock_bus);
}

static void gw_thread_acq(void *arg)
{
    gw_gateway_t *g = (gw_gateway_t *)arg;

    GW_LOGI("ACQ", "acquisition thread started (prio %u, mode=%s)",
            (unsigned)GW_THREAD_PRIO_ACQ,
            (g->mode == GW_LINK_MODE_RTU) ? "MODBUS_RTU" : "MODBUS_TCP");

    while (g->running) {
        uint32_t now = gw_port_tick_ms();
        uint32_t i;
        bool     did_work = false;

        /* 轮询游标保证公平：每次只发一个请求, 遍历所有在线设备 */
        for (i = 0u; i < GW_MAX_DEVICES; i++) {
            uint32_t     idx = (g->poll_cursor + i) % GW_MAX_DEVICES;
            gw_device_t *d   = &g->devm.devices[idx];

            if (!d->used || (d->proto == GW_PROTO_CANOPEN)) {
                continue;
            }
            if (!gw_bus_health_should_poll(&g->bus, d->id, now)) {
                continue;      /* 已被隔离且冷却期未到：跳过, 避免拖垮总线 */
            }

            gw_acq_poll_device(g, d);
            g->poll_cursor = (idx + 1u) % GW_MAX_DEVICES;
            did_work = true;
            break;
        }

        gw_diag_feed(&g->diag, GW_WDT_BIT_ACQ);
        gw_port_delay_ms(did_work ? 1u : 5u);
    }
}

/* ================================================================== */
/* 线程 3：t_parse —— 帧语义解析与数据模型更新                         */
/* ================================================================== */
static void gw_parse_emit_reports(gw_gateway_t *g, gw_device_t *d, uint32_t now)
{
    const gw_dev_profile_t *p = gw_devm_profile(d->profile);
    uint32_t i;

    if (p == NULL) {
        return;
    }

    for (i = 0u; i < p->field_count; i++) {
        const gw_param_field_t *f = &p->fields[i];
        gw_report_t             r;
        int32_t                 v = 0;

        if ((f->reg < d->reg_base) ||
            ((uint32_t)(f->reg - d->reg_base) >= d->reg_count)) {
            continue;      /* 本次轮询窗口没覆盖这个参数 */
        }
        if (gw_devm_read_param(&g->devm, d->id, f->param_id, &v) != GW_OK) {
            continue;
        }

        r.dev_id   = d->id;
        r.quality  = GW_REPORT_QUALITY_GOOD;
        r.param_id = f->param_id;
        r.value    = v;
        r.ts_ms    = now;

        if (gw_mq_send(g->q_reports, &r, GW_NO_WAIT) != GW_OK) {
            g->stats.report_drops++;      /* 上报队列满：丢新不丢旧 */
        } else {
            g->stats.reports++;
        }
    }
}

static void gw_thread_parse(void *arg)
{
    gw_gateway_t *g = (gw_gateway_t *)arg;
    uint16_t      values[GW_MODBUS_MAX_READ_REGS];

    while (g->running) {
        gw_raw_frame_t msg;

        if (gw_mq_recv(g->q_frames, &msg, 100) != GW_OK) {
            gw_diag_feed(&g->diag, GW_WDT_BIT_PARSE);
            continue;
        }

        {
            uint32_t     now = gw_port_tick_ms();
            gw_device_t *d   = gw_devm_find(&g->devm, msg.dev_id);

            if (d != NULL) {
                uint16_t count = (uint16_t)GW_ARRAY_SIZE(values);

                if (gw_gateway_extract_values(g, msg.adu, msg.len, values, &count) == GW_OK) {
                    gw_bus_action_t act;

                    (void)gw_mutex_take(g->lock_devm, GW_WAIT_FOREVER);
                    (void)gw_devm_apply_read(&g->devm, d, d->reg_base, values,
                                             count, now);
                    (void)gw_mutex_release(g->lock_devm);

                    act = gw_bus_health_report_good(&g->bus, d->id, now);
                    if ((act == GW_BUS_ACTION_RECOVER) ||
                        (act == GW_BUS_ACTION_BUS_RECOVER)) {
                        g->recover_events++;
                        (void)gw_event_send(g->evt, GW_EVT_DEV_RECOVERED);
                    }
                    gw_devm_report_poll_ok(&g->devm, d, now);
                    gw_parse_emit_reports(g, d, now);
                } else {
                    /* 帧能过 CRC 但语义不合法：算非法帧, 计入设备健康度 */
                    (void)gw_bus_health_report(&g->bus, d->id,
                                               GW_ERRCLASS_ILLEGAL_FRAME, now);
                }
            }
        }

        (void)gw_mpool_free(&g->pool_adu, msg.adu);   /* 归还内存池 */
        gw_diag_feed(&g->diag, GW_WDT_BIT_PARSE);
    }
}

/* ================================================================== */
/* 线程 4：t_report —— 上报（低频、可阻塞）                            */
/* ================================================================== */
static void gw_thread_report(void *arg)
{
    gw_gateway_t *g = (gw_gateway_t *)arg;

    while (g->running) {
        gw_report_t r;
        if (gw_mq_recv(g->q_reports, &r, 100) == GW_OK) {
            if (g->report_sink != NULL) {
                (void)g->report_sink(g->report_sink_ctx, &r);
            }
        }
        gw_diag_feed(&g->diag, GW_WDT_BIT_REPORT);
    }
}

/* ================================================================== */
/* 线程 5：t_diag —— 诊断、看门狗、心跳超时、总线健康度                */
/* ================================================================== */
static void gw_thread_diag(void *arg)
{
    gw_gateway_t *g = (gw_gateway_t *)arg;
    uint32_t      cycles = 0u;

    while (g->running) {
        uint32_t now = gw_port_tick_ms();
        uint32_t faults;
        int      offline;

        (void)gw_diag_poll(&g->diag, now);
        (void)gw_bus_health_tick(&g->bus, now);

        faults = gw_hw_get_faults();
        if (faults != 0u) {
            (void)gw_bus_health_hw_fault(&g->bus, faults, now);
        }

        offline = gw_devm_check_timeouts(&g->devm, now);
        if (offline > 0) {
            GW_LOGW("DIAG", "%d device(s) went offline (heartbeat timeout)",
                    offline);
        }

        /* 掉电预警：立即把参数写进 Flash（有超级电容/大电容争取时间） */
        if (gw_hw_power_fail_pending()) {
            (void)gw_event_send(g->evt, GW_EVT_POWER_FAIL);
        }

        cycles++;
        if ((cycles % 200u) == 0u) {
            gw_diag_dump(&g->diag);
        }

        gw_diag_feed(&g->diag, GW_WDT_BIT_DIAG);
        gw_port_delay_ms(10u);
    }
}

/* ================================================================== */
/* 线程 6：t_store —— 参数落盘与离线参数缓存回放                       */
/* ================================================================== */
static void gw_store_flush_cache(gw_gateway_t *g)
{
    gw_param_cache_item_t item;

    /* 先记录所有在线设备, 逐个把缓存里的设定值补发下去 */
    while (gw_devm_cache_pop(&g->devm, &item) == GW_OK) {
        uint8_t  req[GW_MAX_ADU_RTU];
        uint8_t  resp[GW_MAX_ADU_RTU];
        uint32_t req_len = 0u;
        uint32_t resp_len = 0u;
        gw_device_t *d = gw_devm_find(&g->devm, item.dev_id);
        int      rc = GW_ERR_NOT_FOUND;

        if (d == NULL) {
            continue;
        }
        if (gw_mutex_take(g->lock_bus, GW_WAIT_FOREVER) != GW_OK) {
            return;
        }
        if (g->mode == GW_LINK_MODE_RTU) {
            rc = gw_mb_rtu_build_write_single(d->addr, item.reg, item.value, req,
                                              sizeof(req), &req_len);
            if (rc == GW_OK) {
                rc = gw_mb_rtu_master_txn(&g->rtu, req, req_len, resp,
                                          sizeof(resp), &resp_len);
            }
        }
        (void)gw_mutex_release(g->lock_bus);

        if (rc == GW_OK) {
            g->stats.cache_flushed++;
            GW_LOGI("STORE", "cached param replayed: dev=%u reg=0x%04X val=%u",
                    (unsigned)item.dev_id, (unsigned)item.reg,
                    (unsigned)item.value);
        } else {
            /* 回放失败：重新入队, 等下一轮 */
            (void)gw_devm_cache_push(&g->devm, item.dev_id, item.reg,
                                     item.value, gw_port_tick_ms());
            break;
        }
    }
}

static void gw_thread_store(void *arg)
{
    gw_gateway_t *g = (gw_gateway_t *)arg;

    while (g->running) {
        uint32_t flags = gw_event_recv(g->evt,
                                       GW_EVT_PARAM_DIRTY | GW_EVT_POWER_FAIL |
                                       GW_EVT_DEV_RECOVERED | GW_EVT_STOP,
                                       false, 100);

        if ((flags & (GW_EVT_PARAM_DIRTY | GW_EVT_POWER_FAIL)) != 0u) {
            (void)gw_event_clear(g->evt, GW_EVT_PARAM_DIRTY | GW_EVT_POWER_FAIL);
            if (gw_param_save(&g->params, &g->cfg_blob,
                              (uint16_t)sizeof(g->cfg_blob)) == GW_OK) {
                g->stats.param_saves++;
            }
        }

        if ((flags & GW_EVT_DEV_RECOVERED) != 0u) {
            (void)gw_event_clear(g->evt, GW_EVT_DEV_RECOVERED);
            gw_store_flush_cache(g);
        }

        gw_diag_feed(&g->diag, GW_WDT_BIT_STORE);
    }
}

/* ================================================================== */
/* 初始化 / 启动 / 停止                                                */
/* ================================================================== */
int gw_gateway_register_device(gw_gateway_t *g, uint8_t addr, gw_protocol_t proto,
                               uint8_t profile, const char *name)
{
    int id;

    if (g == NULL) {
        return GW_ERR_PARAM;
    }
    id = gw_devm_register(&g->devm, addr, proto, profile, name,
                          gw_port_tick_ms());
    if (id < 0) {
        return id;
    }
    (void)gw_bus_health_register_dev(&g->bus, (uint8_t)id);
    return id;
}

int gw_gateway_init(gw_gateway_t *g, gw_link_t *link, const gw_gateway_cfg_t *cfg)
{
    if ((g == NULL) || (link == NULL) || (cfg == NULL)) {
        return GW_ERR_PARAM;
    }

    memset(g, 0, sizeof(*g));
    g->link = link;
    g->mode = cfg->mode;

    /* ---- 驱动层 ---- */
    if (gw_rb_init(&g->ring, g->ring_storage, GW_RAW_RING_SIZE) != GW_OK) {
        return GW_ERR;
    }

    /* ---- 数据模型层 ---- */
    (void)gw_devm_init(&g->devm);
    (void)gw_bus_health_init(&g->bus);
    (void)gw_param_store_init(&g->params);

    if (gw_param_load(&g->params, &g->cfg_blob, (uint16_t)sizeof(g->cfg_blob),
                      &(uint16_t){0}) != GW_OK) {
        /* 首次上电 / 双槽都坏：用出厂默认值并立刻落盘 */
        gw_param_defaults(&g->cfg_blob);
        GW_LOGW("MAIN", "no valid parameter image, loading factory defaults");
    }

    /* ---- 服务层：同步对象 ---- */
    g->q_frames  = gw_mq_create("q_frames", sizeof(gw_raw_frame_t), GW_FRAME_QUEUE_DEPTH);
    g->q_reports = gw_mq_create("q_reports", sizeof(gw_report_t), GW_REPORT_QUEUE_DEPTH);
    g->q_store   = gw_mq_create("q_store", sizeof(gw_raw_frame_t), GW_STORE_QUEUE_DEPTH);
    g->evt       = gw_event_create("gw_evt");
    g->sem_rx    = gw_sem_create("sem_rx", 0u, 64u);
    g->lock_devm = gw_mutex_create("lock_devm", true);   /* 优先级继承 */
    g->lock_bus  = gw_mutex_create("lock_bus", true);

    if ((g->q_frames == NULL) || (g->q_reports == NULL) || (g->q_store == NULL) ||
        (g->evt == NULL) || (g->sem_rx == NULL) || (g->lock_devm == NULL) ||
        (g->lock_bus == NULL)) {
        return GW_ERR_NOMEM;
    }

    /* ---- 服务层：内存池 ---- */
    if ((gw_mpool_init(&g->pool_frame, "mp_frame", g->pool_frame_mem,
                       GW_POOL_FRAME_BLOCK, GW_POOL_FRAME_COUNT) != GW_OK) ||
        (gw_mpool_init(&g->pool_adu, "mp_adu", g->pool_adu_mem,
                       GW_POOL_ADU_BLOCK, GW_POOL_ADU_COUNT) != GW_OK) ||
        (gw_mpool_init(&g->pool_param, "mp_param", g->pool_param_mem,
                       GW_POOL_PARAM_BLOCK, GW_POOL_PARAM_COUNT) != GW_OK)) {
        return GW_ERR_NOMEM;
    }

    /* ---- 协议适配层 ---- */
    if (g->mode == GW_LINK_MODE_RTU) {
        if (gw_mb_rtu_master_init(&g->rtu, link, cfg->resp_timeout_ms,
                                  cfg->retries) != GW_OK) {
            return GW_ERR;
        }
        gw_mb_rtu_master_set_frame_source(&g->rtu, gw_gateway_frame_source, g);
        if (gw_frame_sync_init(&g->fs, g->fs_buf, sizeof(g->fs_buf), GW_T35_US,
                               gw_mb_rtu_expected_len, gw_gateway_fs_cb, g) != GW_OK) {
            return GW_ERR;
        }
    } else {
        if (gw_mb_tcp_master_init(&g->tcp, link, cfg->resp_timeout_ms,
                                  cfg->retries) != GW_OK) {
            return GW_ERR;
        }
        gw_mb_tcp_master_set_frame_source(&g->tcp, gw_gateway_frame_source, g);
        gw_mb_tcp_asm_init(&g->tcp_asm);
    }

    /* ---- 服务层：诊断与看门狗 ---- */
    (void)gw_diag_init(&g->diag, GW_WDT_TIMEOUT_MS);
    gw_diag_attach_bus(&g->diag, &g->bus);

    return GW_OK;
}

void gw_gateway_set_report_sink(gw_gateway_t *g, gw_report_sink_fn fn, void *ctx)
{
    if (g != NULL) {
        g->report_sink     = fn;
        g->report_sink_ctx = ctx;
    }
}

void gw_gateway_set_stats_sink(gw_gateway_t *g, gw_stats_sink_fn fn, void *ctx)
{
    if (g != NULL) {
        g->stats_sink     = fn;
        g->stats_sink_ctx = ctx;
    }
}

int gw_gateway_start(gw_gateway_t *g)
{
    if (g == NULL) {
        return GW_ERR_PARAM;
    }

    g->running        = true;
    g->stop_requested = false;

    g->t_irq = gw_thread_create(GW_THREAD_NAME_IRQ, gw_thread_irq, g,
                                GW_THREAD_STACK_IRQ, GW_THREAD_PRIO_IRQ);
    g->t_acq = gw_thread_create(GW_THREAD_NAME_ACQ, gw_thread_acq, g,
                                GW_THREAD_STACK_ACQ, GW_THREAD_PRIO_ACQ);
    g->t_parse = gw_thread_create(GW_THREAD_NAME_PARSE, gw_thread_parse, g,
                                  GW_THREAD_STACK_PARSE, GW_THREAD_PRIO_PARSE);
    g->t_report = gw_thread_create(GW_THREAD_NAME_REPORT, gw_thread_report, g,
                                   GW_THREAD_STACK_REPORT, GW_THREAD_PRIO_REPORT);
    g->t_diag = gw_thread_create(GW_THREAD_NAME_DIAG, gw_thread_diag, g,
                                 GW_THREAD_STACK_DIAG, GW_THREAD_PRIO_DIAG);
    g->t_store = gw_thread_create(GW_THREAD_NAME_STORE, gw_thread_store, g,
                                  GW_THREAD_STACK_STORE, GW_THREAD_PRIO_STORE);

    if ((g->t_irq == NULL) || (g->t_acq == NULL) || (g->t_parse == NULL) ||
        (g->t_report == NULL) || (g->t_diag == NULL) || (g->t_store == NULL)) {
        g->running = false;
        return GW_ERR_NOMEM;
    }

    (void)gw_diag_register_thread(&g->diag, g->t_irq, GW_THREAD_NAME_IRQ,
                                  GW_WDT_BIT_IRQ, GW_THREAD_PRIO_IRQ, GW_THREAD_STACK_IRQ);
    (void)gw_diag_register_thread(&g->diag, g->t_acq, GW_THREAD_NAME_ACQ,
                                  GW_WDT_BIT_ACQ, GW_THREAD_PRIO_ACQ, GW_THREAD_STACK_ACQ);
    (void)gw_diag_register_thread(&g->diag, g->t_parse, GW_THREAD_NAME_PARSE,
                                  GW_WDT_BIT_PARSE, GW_THREAD_PRIO_PARSE, GW_THREAD_STACK_PARSE);
    (void)gw_diag_register_thread(&g->diag, g->t_report, GW_THREAD_NAME_REPORT,
                                  GW_WDT_BIT_REPORT, GW_THREAD_PRIO_REPORT, GW_THREAD_STACK_REPORT);
    (void)gw_diag_register_thread(&g->diag, g->t_diag, GW_THREAD_NAME_DIAG,
                                  GW_WDT_BIT_DIAG, GW_THREAD_PRIO_DIAG, GW_THREAD_STACK_DIAG);
    (void)gw_diag_register_thread(&g->diag, g->t_store, GW_THREAD_NAME_STORE,
                                  GW_WDT_BIT_STORE, GW_THREAD_PRIO_STORE, GW_THREAD_STACK_STORE);

    (void)gw_event_send(g->evt, GW_EVT_START);
    GW_LOGI("MAIN", "gateway started: %u device(s), link=%s, mode=%s",
            (unsigned)g->devm.device_count, g->link->name,
            (g->mode == GW_LINK_MODE_RTU) ? "MODBUS_RTU" : "MODBUS_TCP");
    return GW_OK;
}

void gw_gateway_stop(gw_gateway_t *g)
{
    if (g == NULL) {
        return;
    }
    g->stop_requested = true;
    g->running        = false;
    (void)gw_event_send(g->evt, GW_EVT_STOP);

    (void)gw_thread_join(g->t_irq, 2000);
    (void)gw_thread_join(g->t_acq, 2000);
    (void)gw_thread_join(g->t_parse, 2000);
    (void)gw_thread_join(g->t_report, 2000);
    (void)gw_thread_join(g->t_diag, 2000);
    (void)gw_thread_join(g->t_store, 2000);

    gw_diag_dump(&g->diag);
    GW_LOGI("MAIN", "gateway stopped");
}

/* 输出结构化统计：verify_protocol.py 直接解析这一行 JSON */
void gw_gateway_dump_stats(gw_gateway_t *g)
{
    char             line[3072];
    gw_mpool_stats_t mp_adu;
    gw_mpool_stats_t mp_frame;
    gw_mutex_pi_stats_t pi;
    gw_err_stats_t   es;

    if (g == NULL) {
        return;
    }

    gw_mpool_stats(&g->pool_adu, &mp_adu);
    gw_mpool_stats(&g->pool_frame, &mp_frame);
    gw_mutex_pi_stats(g->lock_devm, &pi);
    gw_bus_health_get_stats(&g->bus, &es);

    /* 单行 JSON, 便于 Python 端 split 后 json.loads */
    snprintf(line, sizeof(line),
           "GW_STATS_JSON {\"rx_bytes\":%llu,\"tx_frames\":%llu,"
           "\"frames_delivered\":%llu,\"frames_crc_error\":%llu,"
           "\"frames_short\":%llu,\"resync\":%llu,"
           "\"reports\":%llu,\"report_drops\":%llu,\"frame_drops\":%llu,"
           "\"polls_attempted\":%llu,\"polls_ok\":%llu,\"polls_failed\":%llu,"
           "\"retries\":%u,\"timeouts\":%u,\"master_crc_errors\":%u,"
           "\"exceptions\":%u,\"isolate_total\":%u,\"recover_total\":%u,"
           "\"degrade_total\":%u,\"bus_faults\":%u,"
           "\"err_crc\":%u,\"err_timeout\":%u,\"err_illegal\":%u,"
           "\"err_exception\":%u,\"err_bus_short\":%u,"
           "\"pool_adu_used\":%u,\"pool_adu_peak\":%u,\"pool_adu_fail\":%u,"
           "\"pool_frame_peak\":%u,\"param_saves\":%llu,\"cache_flushed\":%llu,"
           "\"pi_inherit\":%u,\"pi_restore\":%u,\"pi_contend\":%u,"
           "\"stack_acq\":%d,\"stack_parse\":%d,\"stack_report\":%d,"
           "\"stack_diag\":%d,\"stack_store\":%d,\"stack_irq\":%d,"
           "\"wdt_misses\":%u,\"stack_overflows\":%u}\n",
           (unsigned long long)g->stats.rx_bytes,
           (unsigned long long)g->stats.tx_frames,
           (unsigned long long)g->stats.frames_delivered,
           (unsigned long long)g->stats.frames_crc_error,
           (unsigned long long)g->stats.frames_short,
           (unsigned long long)g->stats.resync_count,
           (unsigned long long)g->stats.reports,
           (unsigned long long)g->stats.report_drops,
           (unsigned long long)g->stats.frame_drops,
           (unsigned long long)g->stats.polls_attempted,
           (unsigned long long)g->stats.polls_ok,
           (unsigned long long)g->stats.polls_failed,
           (unsigned)g->rtu.retry_count, (unsigned)g->rtu.timeouts,
           (unsigned)g->rtu.crc_errors, (unsigned)g->rtu.exceptions,
           (unsigned)g->bus.isolate_total, (unsigned)g->bus.recover_total,
           (unsigned)g->bus.degrade_total, (unsigned)g->bus.bus_fault_count,
           (unsigned)es.per_class[GW_ERRCLASS_CRC],
           (unsigned)es.per_class[GW_ERRCLASS_TIMEOUT],
           (unsigned)es.per_class[GW_ERRCLASS_ILLEGAL_FRAME],
           (unsigned)es.per_class[GW_ERRCLASS_EXCEPTION],
           (unsigned)es.per_class[GW_ERRCLASS_BUS_SHORT],
           (unsigned)mp_adu.max_used_blocks, (unsigned)mp_adu.peak_bytes,
           (unsigned)mp_adu.alloc_fail,
           (unsigned)mp_frame.max_used_blocks,
           (unsigned long long)g->stats.param_saves,
           (unsigned long long)g->stats.cache_flushed,
           (unsigned)pi.inherit_count, (unsigned)pi.restore_count,
           (unsigned)pi.contend_count,
           (int)gw_diag_stack_peak(&g->diag, GW_THREAD_NAME_ACQ),
           (int)gw_diag_stack_peak(&g->diag, GW_THREAD_NAME_PARSE),
           (int)gw_diag_stack_peak(&g->diag, GW_THREAD_NAME_REPORT),
           (int)gw_diag_stack_peak(&g->diag, GW_THREAD_NAME_DIAG),
           (int)gw_diag_stack_peak(&g->diag, GW_THREAD_NAME_STORE),
           (int)gw_diag_stack_peak(&g->diag, GW_THREAD_NAME_IRQ),
           (unsigned)g->diag.wdt_miss_count,
           (unsigned)g->diag.overflow_count);

    if (g->stats_sink != NULL) {
        g->stats_sink(g->stats_sink_ctx, line);
    } else {
        fputs(line, stdout);
    }
}
