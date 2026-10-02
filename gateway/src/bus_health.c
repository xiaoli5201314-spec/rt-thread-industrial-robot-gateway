/*
 * bus_health.c - 总线错误分级处理实现
 *
 * 判决规则（与 README 中的策略表一一对应）：
 *
 *   错误类别        计数方式            阈值            动作
 *   ------------------------------------------------------------------
 *   非法帧          连续计数            >= 5            隔离设备
 *   CRC 错          连续计数            >= 5            隔离设备
 *   超时/掉线       连续计数            >= 3            隔离设备
 *   从站异常响应    仅统计              不适用          不隔离（有效答复）
 *   总线短路        硬件位图            立即            整条总线 FAULT
 *   栈溢出          一次性              立即            记录 + 看门狗上报
 *
 *   * 任一正常帧到达即清零对应的连续计数（滑窗失败）；
 *   * 被隔离设备在冷却期（3s）内不轮询, 冷却期满后允许低频探测,
 *     连续 3 个正常帧 -> 恢复轮询并记录 recover 事件。
 */
#include "bus_health.h"
#include "port_rtos.h"
#include "port_hw.h"
#include "gw_log.h"

#include <string.h>

#define GW_ILLEGAL_ISOLATE_N   5u

const char *gw_err_class_str(gw_err_class_t c)
{
    switch (c) {
    case GW_ERRCLASS_NONE:           return "NONE";
    case GW_ERRCLASS_ILLEGAL_FRAME:  return "ILLEGAL_FRAME";
    case GW_ERRCLASS_CRC:            return "CRC";
    case GW_ERRCLASS_TIMEOUT:        return "TIMEOUT";
    case GW_ERRCLASS_BUS_SHORT:      return "BUS_SHORT";
    case GW_ERRCLASS_EXCEPTION:      return "EXCEPTION";
    case GW_ERRCLASS_STACK_OVERFLOW: return "STACK_OVERFLOW";
    default:                         return "UNKNOWN";
    }
}

bool gw_err_class_is_fatal(gw_err_class_t c)
{
    /* 从站异常响应是"从站明确答复了", 属于协议层正常交互, 不该触发隔离,
     * 否则一个不支持的功能码就会把好设备拉黑。 */
    return (c != GW_ERRCLASS_NONE) && (c != GW_ERRCLASS_EXCEPTION);
}

const char *gw_bus_action_str(gw_bus_action_t a)
{
    switch (a) {
    case GW_BUS_ACTION_NONE:        return "NONE";
    case GW_BUS_ACTION_WARN:        return "WARN";
    case GW_BUS_ACTION_DEGRADE:     return "DEGRADE";
    case GW_BUS_ACTION_ISOLATE:     return "ISOLATE";
    case GW_BUS_ACTION_RECOVER:     return "RECOVER";
    case GW_BUS_ACTION_BUS_FAULT:   return "BUS_FAULT";
    case GW_BUS_ACTION_BUS_RECOVER: return "BUS_RECOVER";
    default:                        return "UNKNOWN";
    }
}

int gw_bus_health_init(gw_bus_health_t *bh)
{
    if (bh == NULL) {
        return GW_ERR_PARAM;
    }
    memset(bh, 0, sizeof(*bh));
    bh->state           = GW_BUS_OK;
    bh->window_start_ms = gw_port_tick_ms();
    return GW_OK;
}

gw_bus_dev_health_t *gw_bus_health_dev(gw_bus_health_t *bh, uint8_t dev_id)
{
    uint32_t i;

    if (bh == NULL) {
        return NULL;
    }
    for (i = 0u; i < GW_MAX_DEVICES; i++) {
        if (bh->devs[i].used && (bh->devs[i].dev_id == dev_id)) {
            return &bh->devs[i];
        }
    }
    return NULL;
}

int gw_bus_health_register_dev(gw_bus_health_t *bh, uint8_t dev_id)
{
    uint32_t i;

    if (bh == NULL) {
        return GW_ERR_PARAM;
    }
    if (gw_bus_health_dev(bh, dev_id) != NULL) {
        return GW_ERR_STATE;
    }
    for (i = 0u; i < GW_MAX_DEVICES; i++) {
        if (!bh->devs[i].used) {
            memset(&bh->devs[i], 0, sizeof(bh->devs[i]));
            bh->devs[i].used   = true;
            bh->devs[i].dev_id = dev_id;
            bh->dev_count++;
            return GW_OK;
        }
    }
    return GW_ERR_FULL;
}

static void bh_log_event(gw_bus_health_t *bh, uint8_t dev_id,
                         gw_err_class_t cls, gw_bus_action_t action,
                         uint32_t counter, uint32_t now_ms)
{
    gw_bus_event_t *ev = &bh->events[bh->event_head];

    ev->timestamp_ms = now_ms;
    ev->dev_id       = dev_id;
    ev->err_class    = cls;
    ev->action       = action;
    ev->counter      = counter;

    bh->event_head = (bh->event_head + 1u) % GW_EVENT_LOG_DEPTH;
    bh->event_total++;
}

static void bh_set_state(gw_bus_health_t *bh, gw_bus_state_t st)
{
    if (bh->state != st) {
        GW_LOGW("BUS", "bus state %s -> %s",
                gw_bus_state_str(bh->state), gw_bus_state_str(st));
        bh->state = st;
    }
}

gw_bus_action_t gw_bus_health_report(gw_bus_health_t *bh, uint8_t dev_id,
                                     gw_err_class_t cls, uint32_t now_ms)
{
    gw_bus_dev_health_t *d;
    gw_bus_action_t      action = GW_BUS_ACTION_NONE;
    uint32_t             counter = 0u;

    if ((bh == NULL) || (cls >= GW_ERRCLASS_COUNT) || (cls == GW_ERRCLASS_NONE)) {
        return GW_BUS_ACTION_NONE;
    }

    bh->stats.total++;
    bh->stats.per_class[cls]++;
    bh->window_errors++;
    bh->window_total++;

    /* --- 总线级故障：短路/过流立即停车, 不针对单台设备 --- */
    if (cls == GW_ERRCLASS_BUS_SHORT) {
        if (!bh->bus_fault) {
            bh->bus_fault       = true;
            bh->bus_fault_count++;
            bh->bus_fault_ts_ms = now_ms;
            bh->bus_recover_good = 0u;
            bh_set_state(bh, GW_BUS_FAULT);
            bh_log_event(bh, 0u, cls, GW_BUS_ACTION_BUS_FAULT, 0u, now_ms);
        }
        GW_LOGE("BUS", "BUS FAULT: short/over-current detected, TX suspended");
        return GW_BUS_ACTION_BUS_FAULT;
    }

    if (cls == GW_ERRCLASS_STACK_OVERFLOW) {
        bh_log_event(bh, dev_id, cls, GW_BUS_ACTION_WARN, 0u, now_ms);
        GW_LOGE("BUS", "thread stack overflow reported (dev=%u)", (unsigned)dev_id);
        return GW_BUS_ACTION_WARN;
    }

    d = gw_bus_health_dev(bh, dev_id);
    if (d == NULL) {
        return GW_BUS_ACTION_NONE;
    }

    /* 已隔离设备再次出错：不重复产生隔离事件, 但要刷新冷却时间,
     * 使探测速率被限制为"每冷却周期一次", 避免隔离设备霸占总线 */
    if (d->isolated) {
        d->total_errors++;
        d->consec_good = 0u;
        d->isolate_ts_ms = now_ms;
        return GW_BUS_ACTION_NONE;
    }

    d->total_errors++;
    d->consec_good = 0u;

    switch (cls) {
    case GW_ERRCLASS_CRC:
        d->consec_crc++;
        d->consec_illegal = 0u;
        d->consec_timeout = 0u;
        counter = d->consec_crc;
        if (!d->isolated && (d->consec_crc >= GW_CRC_ERR_ISOLATE_N)) {
            action = GW_BUS_ACTION_ISOLATE;
        }
        break;

    case GW_ERRCLASS_ILLEGAL_FRAME:
        d->consec_illegal++;
        d->consec_crc = 0u;
        d->consec_timeout = 0u;
        counter = d->consec_illegal;
        if (!d->isolated && (d->consec_illegal >= GW_ILLEGAL_ISOLATE_N)) {
            action = GW_BUS_ACTION_ISOLATE;
        }
        break;

    case GW_ERRCLASS_TIMEOUT:
        d->consec_timeout++;
        d->consec_crc = 0u;
        d->consec_illegal = 0u;
        counter = d->consec_timeout;
        if (!d->isolated && (d->consec_timeout >= GW_TIMEOUT_ISOLATE_N)) {
            action = GW_BUS_ACTION_ISOLATE;
        }
        break;

    case GW_ERRCLASS_EXCEPTION:
        /* 只计数, 不清零也不触发隔离 */
        counter = d->total_errors;
        return GW_BUS_ACTION_NONE;

    default:
        break;
    }

    if (action == GW_BUS_ACTION_ISOLATE) {
        d->isolated       = true;
        d->isolate_ts_ms  = now_ms;
        d->isolate_count++;
        d->consec_good    = 0u;
        bh->isolate_total++;
        bh_set_state(bh, GW_BUS_ISOLATED);
        bh_log_event(bh, dev_id, cls, action, counter, now_ms);
        GW_LOGW("BUS", "device %u ISOLATED after %u consecutive %s errors",
                (unsigned)dev_id, (unsigned)counter, gw_err_class_str(cls));
        return action;
    }

    if ((action == GW_BUS_ACTION_NONE) && (counter > 0u)) {
        bh_log_event(bh, dev_id, cls, GW_BUS_ACTION_WARN, counter, now_ms);
        if (bh->state == GW_BUS_OK) {
            bh_set_state(bh, GW_BUS_WARN);
        }
        action = GW_BUS_ACTION_WARN;
    }
    return action;
}

gw_bus_action_t gw_bus_health_report_good(gw_bus_health_t *bh, uint8_t dev_id,
                                          uint32_t now_ms)
{
    gw_bus_dev_health_t *d;
    gw_bus_action_t      action = GW_BUS_ACTION_NONE;

    if (bh == NULL) {
        return GW_BUS_ACTION_NONE;
    }

    bh->window_total++;

    /* 总线故障恢复：硬件故障位清零后, 连续 N 个好帧才解除 FAULT */
    if (bh->bus_fault) {
        bh->bus_recover_good++;
        if (bh->bus_recover_good >= GW_RECOVER_GOOD_N) {
            bh->bus_fault = false;
            bh_set_state(bh, GW_BUS_OK);
            bh_log_event(bh, 0u, GW_ERRCLASS_NONE, GW_BUS_ACTION_BUS_RECOVER,
                         bh->bus_recover_good, now_ms);
            GW_LOGI("BUS", "bus fault cleared after %u good frames",
                    (unsigned)bh->bus_recover_good);
            return GW_BUS_ACTION_BUS_RECOVER;
        }
        return GW_BUS_ACTION_NONE;
    }

    d = gw_bus_health_dev(bh, dev_id);
    if (d == NULL) {
        return GW_BUS_ACTION_NONE;
    }

    d->total_good++;
    d->consec_good++;
    d->consec_crc     = 0u;
    d->consec_illegal = 0u;
    d->consec_timeout = 0u;

    if (d->isolated && (d->consec_good >= GW_RECOVER_GOOD_N)) {
        d->isolated = false;
        d->isolate_ts_ms = 0u;
        d->recover_count++;
        bh->recover_total++;
        action = GW_BUS_ACTION_RECOVER;
        bh_log_event(bh, dev_id, GW_ERRCLASS_NONE, action, d->consec_good, now_ms);
        GW_LOGI("BUS", "device %u RECOVERED after %u consecutive good frames",
                (unsigned)dev_id, (unsigned)d->consec_good);
        /* 若没有其它设备仍处于隔离, 总线回到 OK */
        {
            uint32_t i;
            bool any_isolated = false;
            for (i = 0u; i < GW_MAX_DEVICES; i++) {
                if (bh->devs[i].used && bh->devs[i].isolated) {
                    any_isolated = true;
                    break;
                }
            }
            if (!any_isolated) {
                bh_set_state(bh, GW_BUS_OK);
            }
        }
    }
    return action;
}

gw_bus_action_t gw_bus_health_hw_fault(gw_bus_health_t *bh, uint32_t fault_mask,
                                       uint32_t now_ms)
{
    if (bh == NULL) {
        return GW_BUS_ACTION_NONE;
    }
    bh->hw_fault_mask = fault_mask;

    if ((fault_mask & (GW_HW_FAULT_RS485_SHORT | GW_HW_FAULT_CAN_BUS_OFF |
                       GW_HW_FAULT_CAN_OVERCUR)) != 0u) {
        return gw_bus_health_report(bh, 0u, GW_ERRCLASS_BUS_SHORT, now_ms);
    }
    return GW_BUS_ACTION_NONE;
}

bool gw_bus_health_is_isolated(gw_bus_health_t *bh, uint8_t dev_id)
{
    gw_bus_dev_health_t *d = gw_bus_health_dev(bh, dev_id);
    return (d != NULL) && d->isolated;
}

bool gw_bus_health_should_poll(gw_bus_health_t *bh, uint8_t dev_id, uint32_t now_ms)
{
    gw_bus_dev_health_t *d = gw_bus_health_dev(bh, dev_id);

    if (bh == NULL) {
        return false;
    }
    if (bh->bus_fault) {
        return false;          /* 总线级故障期间谁都不发 */
    }
    if (d == NULL) {
        return false;
    }
    if (!d->isolated) {
        return true;
    }
    /* 隔离期内的探测：冷却期满后允许试探性恢复, 避免"一次干扰永久掉线" */
    if ((now_ms - d->isolate_ts_ms) >= GW_ISOLATE_COOLDOWN_MS) {
        d->probe_count++;
        return true;
    }
    return false;
}

gw_bus_state_t gw_bus_health_state(gw_bus_health_t *bh)
{
    return (bh != NULL) ? bh->state : GW_BUS_OK;
}

void gw_bus_health_get_stats(gw_bus_health_t *bh, gw_err_stats_t *out)
{
    if ((bh == NULL) || (out == NULL)) {
        return;
    }
    *out = bh->stats;
}

bool gw_bus_health_get_event(gw_bus_health_t *bh, uint32_t back_index,
                             gw_bus_event_t *out)
{
    uint32_t idx;

    if ((bh == NULL) || (out == NULL) || (back_index >= bh->event_total)) {
        return false;
    }
    if (back_index >= GW_EVENT_LOG_DEPTH) {
        return false;
    }
    idx = (bh->event_head + GW_EVENT_LOG_DEPTH - 1u - back_index) % GW_EVENT_LOG_DEPTH;
    *out = bh->events[idx];
    return true;
}

gw_bus_state_t gw_bus_health_tick(gw_bus_health_t *bh, uint32_t now_ms)
{
    uint32_t window;

    if (bh == NULL) {
        return GW_BUS_OK;
    }

    window = now_ms - bh->window_start_ms;
    if (window < GW_BUS_ERR_WINDOW_MS) {
        return bh->state;
    }

    /* 错误率 = 错误数 / 总帧数, 放大 100 倍保存为整数（避免浮点） */
    bh->ratio_x100 = (bh->window_total > 0u)
                   ? ((bh->window_errors * 10000u) / bh->window_total)
                   : 0u;

    if (!bh->bus_fault) {
        if (bh->ratio_x100 > GW_BUS_DEGRADE_RATIO_X100) {
            if (bh->state != GW_BUS_ISOLATED) {
                bh->degrade_total++;
                bh_set_state(bh, GW_BUS_DEGRADED);
            }
        } else if ((bh->state == GW_BUS_DEGRADED) || (bh->state == GW_BUS_WARN)) {
            if (bh->ratio_x100 == 0u) {
                bh_set_state(bh, GW_BUS_OK);
            }
        }
    }

    bh->window_start_ms = now_ms;
    bh->window_errors   = 0u;
    bh->window_total    = 0u;
    return bh->state;
}
