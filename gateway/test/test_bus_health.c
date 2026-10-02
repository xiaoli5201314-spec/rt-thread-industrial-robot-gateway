/*
 * test_bus_health.c - 总线错误分级与设备隔离/恢复测试
 *
 * 验收标准：连续 N 次 CRC 错触发设备隔离, 恢复帧到达后重新纳入轮询。
 */
#include "test_util.h"
#include "bus_health.h"
#include "port_rtos.h"
#include "port_hw.h"

#include <string.h>

void test_bus_health(void)
{
    gw_bus_health_t bh;
    uint32_t        now = 100000u;
    gw_bus_action_t act;

    GW_SUITE("总线健康度分级 / 隔离 / 恢复");

    GW_CASE("初始化与设备注册");
    GW_ASSERT_EQ_INT(gw_bus_health_init(&bh), GW_OK);
    GW_ASSERT_EQ_INT(gw_bus_health_state(&bh), (int)GW_BUS_OK);
    GW_ASSERT_EQ_INT(gw_bus_health_register_dev(&bh, 1u), GW_OK);
    GW_ASSERT_EQ_INT(gw_bus_health_register_dev(&bh, 2u), GW_OK);
    GW_ASSERT_EQ_INT(gw_bus_health_register_dev(&bh, 1u), GW_ERR_STATE);
    GW_ASSERT_EQ_INT(bh.dev_count, 2u);

    GW_CASE("单次错误只告警, 不隔离");
    act = gw_bus_health_report(&bh, 1u, GW_ERRCLASS_CRC, now);
    GW_ASSERT_EQ_INT(act, (int)GW_BUS_ACTION_WARN);
    GW_ASSERT_EQ_INT(gw_bus_health_state(&bh), (int)GW_BUS_WARN);
    GW_ASSERT(!gw_bus_health_is_isolated(&bh, 1u));
    GW_ASSERT(gw_bus_health_should_poll(&bh, 1u, now));

    GW_CASE("连续 5 次 CRC 错 -> 设备被隔离（阈值 GW_CRC_ERR_ISOLATE_N=5）");
    {
        int i;
        for (i = 0; i < 4; i++) {
            now += 10u;
            act = gw_bus_health_report(&bh, 1u, GW_ERRCLASS_CRC, now);
        }
        /* 第 5 次（含前面 1 次共 5 次）触发隔离 */
        GW_ASSERT_EQ_INT(act, (int)GW_BUS_ACTION_ISOLATE);
        GW_ASSERT(gw_bus_health_is_isolated(&bh, 1u));
        GW_ASSERT_EQ_INT(gw_bus_health_state(&bh), (int)GW_BUS_ISOLATED);
        GW_ASSERT_EQ_INT(bh.isolate_total, 1u);
        {
            gw_bus_dev_health_t *d = gw_bus_health_dev(&bh, 1u);
            GW_ASSERT(d != NULL);
            GW_ASSERT_EQ_INT(d->consec_crc, 5u);
            GW_ASSERT_EQ_INT(d->isolate_count, 1u);
        }
    }

    GW_CASE("隔离期内不轮询被隔离设备, 但其它设备照常轮询（单点故障不拖垮链路）");
    GW_ASSERT(!gw_bus_health_should_poll(&bh, 1u, now + 100u));
    GW_ASSERT(gw_bus_health_should_poll(&bh, 2u, now + 100u));

    GW_CASE("冷却期满后允许低频探测（避免一次干扰永久掉线）");
    GW_ASSERT(!gw_bus_health_should_poll(&bh, 1u,
                                         now + GW_ISOLATE_COOLDOWN_MS - 1u));
    GW_ASSERT(gw_bus_health_should_poll(&bh, 1u,
                                        now + GW_ISOLATE_COOLDOWN_MS + 1u));
    {
        gw_bus_dev_health_t *d = gw_bus_health_dev(&bh, 1u);
        GW_ASSERT(d->probe_count >= 1u);
    }

    GW_CASE("探测失败会刷新冷却时间（探测速率受限, 不霸占总线）");
    {
        uint32_t t2 = now + GW_ISOLATE_COOLDOWN_MS + 10u;
        act = gw_bus_health_report(&bh, 1u, GW_ERRCLASS_CRC, t2);
        GW_ASSERT_EQ_INT(act, (int)GW_BUS_ACTION_NONE);   /* 不重复产生隔离事件 */
        GW_ASSERT_EQ_INT(bh.isolate_total, 1u);
        GW_ASSERT(!gw_bus_health_should_poll(&bh, 1u, t2 + 100u));
        GW_ASSERT(gw_bus_health_should_poll(&bh, 1u, t2 + GW_ISOLATE_COOLDOWN_MS + 1u));
    }

    GW_CASE("连续 3 个恢复帧 -> 设备被重新纳入轮询（RECOVER）");
    {
        uint32_t t3 = now + 3u * GW_ISOLATE_COOLDOWN_MS;
        GW_ASSERT_EQ_INT(gw_bus_health_report_good(&bh, 1u, t3),
                         (int)GW_BUS_ACTION_NONE);
        GW_ASSERT_EQ_INT(gw_bus_health_report_good(&bh, 1u, t3 + 10u),
                         (int)GW_BUS_ACTION_NONE);
        act = gw_bus_health_report_good(&bh, 1u, t3 + 20u);
        GW_ASSERT_EQ_INT(act, (int)GW_BUS_ACTION_RECOVER);
        GW_ASSERT(!gw_bus_health_is_isolated(&bh, 1u));
        GW_ASSERT_EQ_INT(bh.recover_total, 1u);
        GW_ASSERT_EQ_INT(gw_bus_health_state(&bh), (int)GW_BUS_OK);
        {
            gw_bus_dev_health_t *d = gw_bus_health_dev(&bh, 1u);
            GW_ASSERT_EQ_INT(d->recover_count, 1u);
            GW_ASSERT_EQ_INT(d->consec_crc, 0u);
        }
        /* 恢复后立刻可以正常轮询 */
        GW_ASSERT(gw_bus_health_should_poll(&bh, 1u, t3 + 30u));
    }

    GW_CASE("设备 2：连续 3 次超时 -> 隔离（阈值 GW_TIMEOUT_ISOLATE_N=3）");
    {
        int i;
        for (i = 0; i < 2; i++) {
            now += 100u;
            act = gw_bus_health_report(&bh, 2u, GW_ERRCLASS_TIMEOUT, now);
            GW_ASSERT_EQ_INT(act, (int)GW_BUS_ACTION_WARN);
        }
        /* 第 3 次连续超时到达阈值 -> 隔离 */
        now += 100u;
        act = gw_bus_health_report(&bh, 2u, GW_ERRCLASS_TIMEOUT, now);
        GW_ASSERT_EQ_INT(act, (int)GW_BUS_ACTION_ISOLATE);
        GW_ASSERT(gw_bus_health_is_isolated(&bh, 2u));
        GW_ASSERT_EQ_INT(bh.isolate_total, 2u);
    }

    GW_CASE("从站异常响应不计入隔离（是有效答复, 不是链路故障）");
    {
        int i;
        for (i = 0; i < 20; i++) {
            now += 10u;
            act = gw_bus_health_report(&bh, 2u, GW_ERRCLASS_EXCEPTION, now);
            GW_ASSERT_EQ_INT(act, (int)GW_BUS_ACTION_NONE);
        }
        GW_ASSERT_EQ_INT(bh.stats.per_class[GW_ERRCLASS_EXCEPTION], 20u);
    }

    GW_CASE("总线短路 -> 整条总线 FAULT, 所有设备停止发送");
    {
        now += 1000u;
        act = gw_bus_health_report(&bh, 0u, GW_ERRCLASS_BUS_SHORT, now);
        GW_ASSERT_EQ_INT(act, (int)GW_BUS_ACTION_BUS_FAULT);
        GW_ASSERT(bh.bus_fault);
        GW_ASSERT_EQ_INT(gw_bus_health_state(&bh), (int)GW_BUS_FAULT);
        GW_ASSERT(!gw_bus_health_should_poll(&bh, 1u, now));      /* 谁都不发 */
        GW_ASSERT(!gw_bus_health_should_poll(&bh, 2u, now));
        GW_ASSERT_EQ_INT(bh.bus_fault_count, 1u);
    }

    GW_CASE("硬件故障位（RS485 短路）走同一通道");
    {
        now += 100u;
        act = gw_bus_health_hw_fault(&bh, GW_HW_FAULT_RS485_SHORT, now);
        GW_ASSERT_EQ_INT(act, (int)GW_BUS_ACTION_BUS_FAULT);
        GW_ASSERT_EQ_INT(bh.hw_fault_mask, GW_HW_FAULT_RS485_SHORT);
    }

    GW_CASE("总线故障解除：连续 3 个好帧后恢复");
    {
        uint32_t t = now + 1000u;
        (void)gw_bus_health_report_good(&bh, 1u, t);
        (void)gw_bus_health_report_good(&bh, 1u, t + 10u);
        act = gw_bus_health_report_good(&bh, 1u, t + 20u);
        GW_ASSERT_EQ_INT(act, (int)GW_BUS_ACTION_BUS_RECOVER);
        GW_ASSERT(!bh.bus_fault);
        GW_ASSERT(gw_bus_health_should_poll(&bh, 1u, t + 30u));
    }

    GW_CASE("错误统计分类正确");
    {
        gw_err_stats_t es;
        gw_bus_health_get_stats(&bh, &es);
        GW_ASSERT_EQ_INT(es.per_class[GW_ERRCLASS_CRC] >= 6u, 1);
        GW_ASSERT_EQ_INT(es.per_class[GW_ERRCLASS_TIMEOUT], 3u);
        GW_ASSERT_EQ_INT(es.per_class[GW_ERRCLASS_EXCEPTION], 20u);
        GW_ASSERT_EQ_INT(es.per_class[GW_ERRCLASS_BUS_SHORT] >= 2u, 1);
        GW_ASSERT(es.total >= 31u);
    }

    GW_CASE("事件日志可按时间倒序读取（现场复盘依据）");
    {
        gw_bus_event_t ev;
        GW_ASSERT(bh.event_total > 0u);
        GW_ASSERT(gw_bus_health_get_event(&bh, 0u, &ev));
        GW_ASSERT(ev.timestamp_ms > 0u);
        GW_ASSERT(ev.action != GW_BUS_ACTION_NONE);
        GW_ASSERT(gw_bus_health_get_event(&bh, 1u, &ev));
        GW_ASSERT(!gw_bus_health_get_event(&bh, 100000u, &ev));
    }

    GW_CASE("错误率窗口：超过 10% 触发 DEGRADED, 清零后回到 OK");
    {
        gw_bus_health_t bh2;
        uint32_t t = 5000u;
        int i;
        GW_ASSERT_EQ_INT(gw_bus_health_init(&bh2), GW_OK);
        GW_ASSERT_EQ_INT(gw_bus_health_register_dev(&bh2, 1u), GW_OK);
        bh2.window_start_ms = t;
        /* 8 次错误 + 8 次成功 = 50% 错误率 */
        for (i = 0; i < 8; i++) {
            (void)gw_bus_health_report(&bh2, 1u, GW_ERRCLASS_CRC, t + (uint32_t)i);
            (void)gw_bus_health_report_good(&bh2, 1u, t + (uint32_t)i + 1u);
        }
        GW_ASSERT_EQ_INT(gw_bus_health_tick(&bh2, t + GW_BUS_ERR_WINDOW_MS + 1u),
                         (int)GW_BUS_DEGRADED);
        GW_ASSERT(bh2.ratio_x100 > GW_BUS_DEGRADE_RATIO_X100);
        GW_ASSERT_EQ_INT(bh2.degrade_total, 1u);

        /* 窗口内全部正常 -> 回到 OK */
        for (i = 0; i < 32; i++) {
            (void)gw_bus_health_report_good(&bh2, 1u,
                                            t + GW_BUS_ERR_WINDOW_MS + 10u + (uint32_t)i);
        }
        GW_ASSERT_EQ_INT(gw_bus_health_tick(&bh2, t + 2u * GW_BUS_ERR_WINDOW_MS + 10u),
                         (int)GW_BUS_OK);
        GW_ASSERT_EQ_INT(bh2.ratio_x100, 0u);
    }

    GW_CASE("非法帧连续 5 次同样触发隔离");
    {
        gw_bus_health_t bh3;
        int i;
        GW_ASSERT_EQ_INT(gw_bus_health_init(&bh3), GW_OK);
        GW_ASSERT_EQ_INT(gw_bus_health_register_dev(&bh3, 7u), GW_OK);
        for (i = 0; i < 4; i++) {
            (void)gw_bus_health_report(&bh3, 7u, GW_ERRCLASS_ILLEGAL_FRAME,
                                       1000u + (uint32_t)i);
        }
        GW_ASSERT_EQ_INT(gw_bus_health_report(&bh3, 7u, GW_ERRCLASS_ILLEGAL_FRAME,
                                              1005u),
                         (int)GW_BUS_ACTION_ISOLATE);
        GW_ASSERT(gw_bus_health_is_isolated(&bh3, 7u));
    }

    GW_CASE("正常帧会清零对应设备的连续错误计数（滑窗失败判定）");
    {
        gw_bus_health_t bh4;
        GW_ASSERT_EQ_INT(gw_bus_health_init(&bh4), GW_OK);
        GW_ASSERT_EQ_INT(gw_bus_health_register_dev(&bh4, 1u), GW_OK);
        (void)gw_bus_health_report(&bh4, 1u, GW_ERRCLASS_CRC, 100u);
        (void)gw_bus_health_report(&bh4, 1u, GW_ERRCLASS_CRC, 101u);
        (void)gw_bus_health_report_good(&bh4, 1u, 102u);
        GW_ASSERT_EQ_INT(gw_bus_health_dev(&bh4, 1u)->consec_crc, 0u);
        (void)gw_bus_health_report(&bh4, 1u, GW_ERRCLASS_CRC, 103u);
        (void)gw_bus_health_report(&bh4, 1u, GW_ERRCLASS_CRC, 104u);
        (void)gw_bus_health_report(&bh4, 1u, GW_ERRCLASS_CRC, 105u);
        GW_ASSERT_EQ_INT(gw_bus_health_report(&bh4, 1u, GW_ERRCLASS_CRC, 106u),
                         (int)GW_BUS_ACTION_WARN);   /* 只有 4 次连续, 未到 5 */
        GW_ASSERT(!gw_bus_health_is_isolated(&bh4, 1u));
    }

    GW_CASE("栈溢出事件走诊断通道, 不参与设备隔离判决");
    {
        gw_bus_health_t bh5;
        GW_ASSERT_EQ_INT(gw_bus_health_init(&bh5), GW_OK);
        GW_ASSERT_EQ_INT(gw_bus_health_register_dev(&bh5, 3u), GW_OK);
        GW_ASSERT_EQ_INT(gw_bus_health_report(&bh5, 3u, GW_ERRCLASS_STACK_OVERFLOW,
                                              200u), (int)GW_BUS_ACTION_WARN);
        GW_ASSERT(!gw_bus_health_is_isolated(&bh5, 3u));
    }
}
