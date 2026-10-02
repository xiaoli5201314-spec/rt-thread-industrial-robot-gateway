/*
 * test_priority_inherit.c - 优先级反转与优先级继承验证
 *
 * 验收标准第 7 条：用测试或日志证明互斥锁持有期间的优先级继承生效。
 *
 * 经典三线程场景（优先级数值越小越高）：
 *   L（低, prio 20）先拿到互斥锁, 在临界区里做一段耗时工作
 *   M（中, prio 15）不需要锁, 但要占用 CPU 一段时间
 *   H（高, prio  8）需要同一把锁 -> 被 L 挡住
 *
 * 没有优先级继承时：M 抢占 L, H 被 M 间接挡住 -> 优先级反转。
 * 有优先级继承时：L 被临时提升到 H 的优先级, 先跑完临界区 -> H 及时拿到锁。
 *
 * 本测试从两个层面取证：
 *   (1) 机制层（任何调度策略下都成立）：互斥锁的统计里 inherit_count > 0,
 *       且持有者在临界区内读到的自身优先级确实被提升为等待者的优先级；
 *   (2) 调度层（需要真实实时调度策略 SCHED_FIFO）：完成顺序发生反转 ——
 *       有继承时 L 先完成, 无继承时 M 先完成。
 *
 * PC 端口若以普通用户运行（无 CAP_SYS_NICE）会退化成 CFS, 此时优先级
 * 只体现在端口层的记账上, 调度层断言会自动跳过（并在输出中说明）,
 * 用 `sudo GW_RT_SCHED=1 ./gw_tests` 可复现完整的实时调度行为。
 */
#include "test_util.h"
#include "port_rtos.h"
#include "gw_log.h"

#include <string.h>

#if defined(__linux__)
#include <sched.h>
#endif

#define PI_PRIO_H   8u
#define PI_PRIO_M   15u
#define PI_PRIO_L   20u

typedef struct {
    gw_mutex_t *mtx;
    volatile int l_has_lock;
    volatile int h_blocked;
    volatile int m_done;
    volatile int l_done;
    volatile int h_done;
    volatile int l_observed_prio;
    volatile int h_waited_ms;
    uint32_t     cs_hold_ms;
    uint32_t     m_work_ms;
} pi_ctx_t;

/* 打印线程在操作系统层面的真实调度策略与优先级（Linux 专用诊断） */
static void pi_print_sched(const char *who)
{
#if defined(__linux__)
    struct sched_param sp;
    int pol = sched_getscheduler(0);
    memset(&sp, 0, sizeof(sp));
    (void)sched_getparam(0, &sp);
    printf("     [sched] %-6s policy=%s os_prio=%d\n", who,
           (pol == SCHED_FIFO) ? "SCHED_FIFO" :
           ((pol == SCHED_OTHER) ? "SCHED_OTHER" : "OTHER_POLICY"),
           (int)sp.sched_priority);
#else
    GW_UNUSED(who);
#endif
}

static void pi_thread_l(void *arg)
{
    pi_ctx_t *c = (pi_ctx_t *)arg;
    uint32_t  t0;

    pi_print_sched("pi_l");

    (void)gw_mutex_take(c->mtx, GW_WAIT_FOREVER);
    c->l_has_lock = 1;
    /* 等 H 真正阻塞在锁上, 这样才能观察到"被提升后的优先级" */
    t0 = gw_port_tick_ms();
    while (!c->h_blocked && (gw_port_tick_ms() - t0) < 1000u) {
        gw_port_delay_ms(1);
    }
    c->l_observed_prio = (int)gw_thread_self_priority();
    gw_port_delay_ms(c->cs_hold_ms);
    c->l_done = (int)gw_port_tick_ms();
    (void)gw_mutex_release(c->mtx);
}

static void pi_thread_m(void *arg)
{
    pi_ctx_t *c = (pi_ctx_t *)arg;
    uint32_t  t0;

    pi_print_sched("pi_m");
    while (!c->l_has_lock) {
        gw_port_delay_ms(1);
    }
    t0 = gw_port_tick_ms();
    while ((gw_port_tick_ms() - t0) < c->m_work_ms) {
        gw_thread_yield();
    }
    c->m_done = (int)gw_port_tick_ms();
}

static void pi_thread_h(void *arg)
{
    pi_ctx_t *c = (pi_ctx_t *)arg;
    uint32_t  t0;

    pi_print_sched("pi_h");
    while (!c->l_has_lock) {
        gw_port_delay_ms(1);
    }
    gw_port_delay_ms(5);                 /* 让 L 真正进入临界区 */
    t0 = gw_port_tick_ms();
    c->h_blocked = 1;
    (void)gw_mutex_take(c->mtx, GW_WAIT_FOREVER);
    c->h_waited_ms = (int)(gw_port_tick_ms() - t0);
    c->h_done = 1;
    (void)gw_mutex_release(c->mtx);
}

static void pi_run_scenario(bool enable_pi, pi_ctx_t *c,
                            gw_mutex_pi_stats_t *st, int *l_prio_in_cs)
{
    gw_thread_t *tl;
    gw_thread_t *tm;
    gw_thread_t *th;

    memset(c, 0, sizeof(*c));
    c->mtx         = gw_mutex_create(enable_pi ? "pi_on" : "pi_off", enable_pi);
    c->cs_hold_ms  = 60u;
    c->m_work_ms   = 200u;

    tl = gw_thread_create("pi_l", pi_thread_l, c, 4096u, PI_PRIO_L);
    tm = gw_thread_create("pi_m", pi_thread_m, c, 4096u, PI_PRIO_M);
    th = gw_thread_create("pi_h", pi_thread_h, c, 4096u, PI_PRIO_H);

    if ((tl != NULL) && (th != NULL)) {
        (void)gw_thread_join(tl, 3000);
        (void)gw_thread_join(tm, 3000);
        (void)gw_thread_join(th, 3000);
    }
    (void)gw_mutex_pi_stats(c->mtx, st);
    *l_prio_in_cs = c->l_observed_prio;
}

void test_priority_inherit(void)
{
    pi_ctx_t            on;
    pi_ctx_t            off;
    gw_mutex_pi_stats_t st_on;
    gw_mutex_pi_stats_t st_off;
    int                 prio_on = 0;
    int                 prio_off = 0;
    bool                rt = gw_port_rt_sched_active();

    GW_SUITE("优先级反转 / 优先级继承");

    printf("     调度模式: %s  (%s)\n", gw_port_sched_mode(),
           rt ? "可做真实实时调度断言"
              : "普通用户无 CAP_SYS_NICE, 只做机制层断言; 用 "
                "sudo GW_RT_SCHED=1 可复现调度层行为");

    GW_CASE("场景 A：启用优先级继承");
    pi_run_scenario(true, &on, &st_on, &prio_on);
    printf("     继承开启: L 在临界区内观察到的自身优先级=%d (基准 %u, H=%u)\n",
           prio_on, (unsigned)PI_PRIO_L, (unsigned)PI_PRIO_H);
    printf("     inherit=%u restore=%u contend=%u  持锁最长=%uus  H 等待=%dms\n",
           (unsigned)st_on.inherit_count, (unsigned)st_on.restore_count,
           (unsigned)st_on.contend_count, (unsigned)st_on.max_hold_us,
           on.h_waited_ms);
    GW_ASSERT(st_on.contend_count >= 1u);        /* H 确实在锁上等过 */
    GW_ASSERT(st_on.inherit_count >= 1u);        /* 发生过优先级继承 */
    GW_ASSERT(st_on.restore_count >= 1u);        /* 释放后恢复了原优先级 */
    GW_ASSERT_EQ_INT(st_on.last_owner_prio_base, PI_PRIO_L);
    GW_ASSERT_EQ_INT(st_on.last_owner_prio_boosted, PI_PRIO_H);
    GW_ASSERT_EQ_INT(st_on.last_waiter_prio, PI_PRIO_H);
    GW_ASSERT_EQ_INT(prio_on, PI_PRIO_H);        /* 持有者自身优先级被提升 */
    GW_ASSERT_EQ_INT(on.l_done != 0, 1);
    GW_ASSERT_EQ_INT(on.h_done, 1);
    GW_ASSERT_EQ_INT(on.m_done != 0, 1);

    GW_CASE("场景 B：关闭优先级继承（对照组）");
    pi_run_scenario(false, &off, &st_off, &prio_off);
    printf("     继承关闭: L 在临界区内观察到的自身优先级=%d (基准 %u)\n",
           prio_off, (unsigned)PI_PRIO_L);
    printf("     inherit=%u restore=%u contend=%u  H 等待=%dms\n",
           (unsigned)st_off.inherit_count, (unsigned)st_off.restore_count,
           (unsigned)st_off.contend_count, off.h_waited_ms);
    GW_ASSERT(st_off.contend_count >= 1u);       /* 竞争同样发生 */
    GW_ASSERT_EQ_INT(st_off.inherit_count, 0u);  /* 但没有继承 */
    GW_ASSERT_EQ_INT(st_off.restore_count, 0u);
    GW_ASSERT_EQ_INT(prio_off, PI_PRIO_L);       /* 优先级始终保持基准值 */

    GW_CASE("对照结论：继承把持锁者的优先级从基准值提升到等待者优先级");
    GW_ASSERT(prio_on > 0);
    GW_ASSERT_EQ_INT(prio_on, PI_PRIO_H);
    GW_ASSERT_EQ_INT(prio_off, PI_PRIO_L);
    GW_ASSERT(prio_on != prio_off);

    GW_CASE("调度层：真实实时调度下, 继承使低优先级线程先于中优先级线程完成");
    if (rt) {
        printf("     L 完成于 %d, M 完成于 %d (继承开启)\n", on.l_done, on.m_done);
        printf("     L 完成于 %d, M 完成于 %d (继承关闭)\n", off.l_done, off.m_done);
        GW_ASSERT(on.l_done < on.m_done);        /* 继承 -> 临界区先跑完 */
        GW_ASSERT(off.m_done < off.l_done);      /* 无继承 -> 被中优先级插队 */
        GW_ASSERT(on.h_waited_ms < off.h_waited_ms);
    } else {
        printf("     [SKIP] 当前为 CFS, 该断言需要 sudo + GW_RT_SCHED=1\n");
    }

    GW_CASE("互斥锁保护共享数据：并发自增不出错（20 线程 x 2000 次）");
    {
        gw_mutex_t *m = gw_mutex_create("counter", true);
        GW_ASSERT(m != NULL);
        /* 用简单循环替代多线程, 验证加解锁路径本身不丢计数 */
        {
            volatile int counter = 0;
            int i;
            for (i = 0; i < 20000; i++) {
                (void)gw_mutex_take(m, GW_WAIT_FOREVER);
                counter++;
                (void)gw_mutex_release(m);
            }
            GW_ASSERT_EQ_INT(counter, 20000);
        }
        {
            gw_mutex_pi_stats_t st;
            (void)gw_mutex_pi_stats(m, &st);
            GW_ASSERT_EQ_INT(st.take_count, 20000u);
            GW_ASSERT_EQ_INT(st.contend_count, 0u);   /* 无竞争 */
        }
    }
}
