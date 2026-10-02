/*
 * diag.c - 诊断服务实现
 *
 * 看门狗采用"位图喂狗"而不是"单点喂狗"：只有全部被监控线程都在窗口内
 * 报到了才算健康。这样某个线程被死循环或死锁挂住时, 复位动作一定会
 * 发生, 而不是被其它线程的喂狗掩盖 —— 这是工业现场最常见的一类
 * "看门狗形同虚设"问题的根因。
 */
#include "diag.h"
#include "gw_log.h"

#include <string.h>

static gw_diag_t *s_diag_singleton = NULL;

/* 端口层栈溢出钩子 -> 诊断服务 */
static void diag_overflow_hook(const char *thread_name, int32_t stack_size)
{
    gw_diag_t *dg = s_diag_singleton;
    uint32_t   i;

    if (dg == NULL) {
        return;
    }
    dg->overflow_count++;
    for (i = 0u; i < GW_DIAG_MAX_THREADS; i++) {
        if (dg->threads[i].used && (dg->threads[i].name != NULL) &&
            (strcmp(dg->threads[i].name, thread_name) == 0)) {
            dg->threads[i].overflow = true;
            break;
        }
    }
    GW_LOGE("DIAG", "STACK OVERFLOW in thread '%s' (stack=%d bytes)",
            (thread_name != NULL) ? thread_name : "?", (int)stack_size);

    if (dg->bus != NULL) {
        (void)gw_bus_health_report(dg->bus, 0u, GW_ERRCLASS_STACK_OVERFLOW,
                                   gw_port_tick_ms());
    }
}

/* 看门狗超时钩子 -> 记录未喂狗掩码并要求复位 */
static void diag_wdt_hook(uint32_t missed_mask, const char *reason)
{
    gw_diag_t *dg = s_diag_singleton;

    if (dg == NULL) {
        return;
    }
    dg->wdt_miss_count++;
    dg->last_missed_mask = missed_mask;
    dg->reset_pending    = true;
    dg->reset_request_count++;
    GW_LOGE("DIAG", "WATCHDOG TIMEOUT: missed_mask=0x%08X (%s) -> reset requested",
            (unsigned)missed_mask, (reason != NULL) ? reason : "-");
}

int gw_diag_init(gw_diag_t *dg, uint32_t wdt_timeout_ms)
{
    if (dg == NULL) {
        return GW_ERR_PARAM;
    }
    memset(dg, 0, sizeof(*dg));
    dg->wdt_timeout_ms = (wdt_timeout_ms == 0u) ? GW_WDT_TIMEOUT_MS : wdt_timeout_ms;

    s_diag_singleton = dg;
    (void)gw_wdt_init(dg->wdt_timeout_ms);
    gw_wdt_set_mask(0u);
    gw_wdt_set_reset_hook(diag_wdt_hook);
    gw_thread_set_overflow_hook(diag_overflow_hook);
    return GW_OK;
}

void gw_diag_attach_bus(gw_diag_t *dg, gw_bus_health_t *bh)
{
    if (dg != NULL) {
        dg->bus = bh;
    }
}

int gw_diag_register_thread(gw_diag_t *dg, gw_thread_t *handle, const char *name,
                            uint32_t wdt_bit, uint8_t priority, uint32_t stack_size)
{
    uint32_t i;
    uint32_t mask = 0u;

    if ((dg == NULL) || (name == NULL)) {
        return GW_ERR_PARAM;
    }
    for (i = 0u; i < GW_DIAG_MAX_THREADS; i++) {
        if (!dg->threads[i].used) {
            dg->threads[i].used            = true;
            dg->threads[i].name            = name;
            dg->threads[i].wdt_bit         = wdt_bit;
            dg->threads[i].priority        = priority;
            dg->threads[i].stack_size      = stack_size;
            dg->threads[i].handle          = handle;
            dg->threads[i].stack_used_peak = 0;
            dg->count++;
            mask = wdt_bit;
            break;
        }
    }
    if (mask == 0u) {
        return GW_ERR_FULL;
    }

    for (i = 0u; i < GW_DIAG_MAX_THREADS; i++) {
        if (dg->threads[i].used) {
            mask |= dg->threads[i].wdt_bit;
        }
    }
    gw_wdt_set_mask(mask);
    return GW_OK;
}

void gw_diag_feed(gw_diag_t *dg, uint32_t wdt_bit)
{
    uint32_t i;

    if (dg == NULL) {
        return;
    }
    for (i = 0u; i < GW_DIAG_MAX_THREADS; i++) {
        if (dg->threads[i].used && (dg->threads[i].wdt_bit == wdt_bit)) {
            dg->threads[i].feed_count++;
            break;
        }
    }
    gw_wdt_feed(wdt_bit);
}

uint32_t gw_diag_poll(gw_diag_t *dg, uint32_t now_ms)
{
    uint32_t i;
    uint32_t missed = 0u;

    GW_UNUSED(now_ms);
    if (dg == NULL) {
        return 0u;
    }

    for (i = 0u; i < GW_DIAG_MAX_THREADS; i++) {
        gw_diag_thread_t *t = &dg->threads[i];
        int32_t           used;

        if (!t->used) {
            continue;
        }
        if (t->handle != NULL) {
            used = gw_thread_stack_used(t->handle);
            if (used < 0) {
                used = 0;
            }
            t->stack_used_last = used;
            if (used > t->stack_used_peak) {
                t->stack_used_peak = used;
            }
            if (gw_thread_stack_overflow(t->handle)) {
                if (!t->overflow) {
                    t->overflow = true;
                    dg->overflow_count++;
                }
            }
        }
    }

    dg->wdt_check_count++;
    if (!gw_wdt_check()) {
        missed           = dg->last_missed_mask;
        for (i = 0u; i < GW_DIAG_MAX_THREADS; i++) {
            if (dg->threads[i].used && ((dg->last_missed_mask & dg->threads[i].wdt_bit) != 0u)) {
                dg->threads[i].missed_count++;
            }
        }
    }
    return missed;
}

int32_t gw_diag_stack_peak(gw_diag_t *dg, const char *name)
{
    uint32_t i;

    if ((dg == NULL) || (name == NULL)) {
        return -1;
    }
    for (i = 0u; i < GW_DIAG_MAX_THREADS; i++) {
        if (dg->threads[i].used && (dg->threads[i].name != NULL) &&
            (strcmp(dg->threads[i].name, name) == 0)) {
            return dg->threads[i].stack_used_peak;
        }
    }
    return -1;
}

void gw_diag_dump(gw_diag_t *dg)
{
    uint32_t i;

    if (dg == NULL) {
        return;
    }
    GW_LOGI("DIAG", "---- thread stack calibration (peak measured) ----");
    GW_LOGI("DIAG", "%-10s %6s %6s %6s %6s", "thread", "prio", "stack",
            "peak", "usage%");
    for (i = 0u; i < GW_DIAG_MAX_THREADS; i++) {
        gw_diag_thread_t *t = &dg->threads[i];
        if (!t->used) {
            continue;
        }
        GW_LOGI("DIAG", "%-10s %6u %6u %6d %5d%%", t->name,
                (unsigned)t->priority, (unsigned)t->stack_size,
                (int)t->stack_used_peak,
                (t->stack_size > 0u)
                    ? (int)((t->stack_used_peak * 100) / (int32_t)t->stack_size)
                    : 0);
    }
    GW_LOGI("DIAG", "wdt: checks=%u misses=%u resets=%u overflows=%u",
            (unsigned)dg->wdt_check_count, (unsigned)dg->wdt_miss_count,
            (unsigned)dg->reset_request_count, (unsigned)dg->overflow_count);
}
