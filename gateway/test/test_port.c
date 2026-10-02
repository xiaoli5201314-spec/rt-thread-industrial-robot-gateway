/*
 * test_port.c - 端口层同步对象测试（消息队列 / 信号量 / 事件标志组 /
 *               互斥锁 / 看门狗 / 栈水位与溢出检测）
 */
#include "test_util.h"
#include "port_rtos.h"
#include "port_hw.h"
#include "diag.h"
#include "mem_pool.h"

#include <string.h>

/* ---------------- 生产者/消费者 ---------------- */
typedef struct {
    gw_mq_t   *q;
    uint32_t   count;
    uint32_t   sum;
    uint32_t   stop_after;
    gw_sem_t  *done;
} prod_ctx_t;

static void prod_thread(void *arg)
{
    prod_ctx_t *c = (prod_ctx_t *)arg;
    uint32_t    i;
    for (i = 0u; i < c->stop_after; i++) {
        uint32_t v = i + 1u;
        while (gw_mq_send(c->q, &v, 200) != GW_OK) {
            /* 队列满: 让消费者先跑 */
        }
    }
    (void)gw_sem_release(c->done);
}

static void cons_thread(void *arg)
{
    prod_ctx_t *c = (prod_ctx_t *)arg;
    for (;;) {
        uint32_t v = 0u;
        if (gw_mq_recv(c->q, &v, 500) != GW_OK) {
            break;
        }
        c->count++;
        c->sum += v;
        if (v == 2000u) {
            break;
        }
    }
    (void)gw_sem_release(c->done);
}

/* ---------------- 事件标志组 ---------------- */
static gw_event_t *g_evt;
static volatile int g_flag_a;
static volatile int g_flag_b;

static void evt_thread(void *arg)
{
    uint32_t flags;
    GW_UNUSED(arg);
    flags = gw_event_recv(g_evt, 0x03u, true, 1000);   /* 与等待 */
    if (flags == 0x03u) {
        g_flag_a = 1;
    }
    flags = gw_event_recv(g_evt, 0xF0u, false, 300);   /* 或等待, 超时 */
    if (flags == 0u) {
        g_flag_b = 1;
    }
}

/* ---------------- 栈溢出检测用线程 ---------------- */
static volatile uint8_t s_sink;

static void deep_recursion(int n)
{
    volatile uint8_t buf[128];
    memset((void *)buf, n, sizeof(buf));
    s_sink = buf[n & 0x7F];
    if (n > 0) {
        deep_recursion(n - 1);
    }
}

static void heavy_thread(void *arg)
{
    GW_UNUSED(arg);
    for (;;) {
        deep_recursion(6);
        gw_port_delay_ms(10);
    }
}

void test_port_objects(void)
{
    GW_SUITE("端口层同步对象");

    gw_port_init();

    GW_CASE("消息队列：跨线程生产者/消费者 2000 条消息不丢不乱序");
    {
        static prod_ctx_t pc;
        gw_mq_t  *q = gw_mq_create("q_t", sizeof(uint32_t), 16u);
        gw_sem_t *done = gw_sem_create("sem_done", 0u, 4u);
        gw_thread_t *tp;
        gw_thread_t *tc;

        GW_ASSERT(q != NULL);
        GW_ASSERT(done != NULL);
        memset(&pc, 0, sizeof(pc));
        pc.q = q;
        pc.done = done;
        pc.stop_after = 2000u;

        tp = gw_thread_create("prod", prod_thread, &pc, 4096u, 12u);
        tc = gw_thread_create("cons", cons_thread, &pc, 4096u, 13u);
        GW_ASSERT(tp != NULL);
        GW_ASSERT(tc != NULL);

        GW_ASSERT_EQ_INT(gw_sem_take(done, 5000), GW_OK);
        GW_ASSERT_EQ_INT(gw_sem_take(done, 5000), GW_OK);
        GW_ASSERT_EQ_INT(pc.count, 2000u);
        GW_ASSERT_EQ_INT(pc.sum, 2000u * 2001u / 2u);   /* 1+2+...+2000 */
        GW_ASSERT_EQ_INT(gw_mq_count(q), 0u);
        GW_ASSERT_EQ_INT(gw_mq_capacity(q), 16u);
    }

    GW_CASE("消息队列：空队列 NO_WAIT 返回 GW_ERR_EMPTY, 带超时返回 TIMEOUT");
    {
        gw_mq_t *q = gw_mq_create("q_t2", sizeof(uint32_t), 4u);
        uint32_t v = 0u;
        uint32_t t0 = gw_port_tick_ms();
        GW_ASSERT_EQ_INT(gw_mq_recv(q, &v, GW_NO_WAIT), GW_ERR_EMPTY);
        GW_ASSERT_EQ_INT(gw_mq_recv(q, &v, 60), GW_ERR_TIMEOUT);
        GW_ASSERT(gw_port_tick_ms() - t0 >= 55u);
    }

    GW_CASE("消息队列：满队列 NO_WAIT 返回 GW_ERR_FULL（上层据此丢弃并计数）");
    {
        gw_mq_t *q = gw_mq_create("q_t3", sizeof(uint32_t), 4u);
        uint32_t v = 0u;
        int i;
        for (i = 0; i < 4; i++) {
            v = (uint32_t)i;
            GW_ASSERT_EQ_INT(gw_mq_send(q, &v, GW_NO_WAIT), GW_OK);
        }
        v = 99u;
        GW_ASSERT_EQ_INT(gw_mq_send(q, &v, GW_NO_WAIT), GW_ERR_FULL);
        GW_ASSERT_EQ_INT(gw_mq_count(q), 4u);
        /* FIFO 顺序 */
        for (i = 0; i < 4; i++) {
            GW_ASSERT_EQ_INT(gw_mq_recv(q, &v, 50), GW_OK);
            GW_ASSERT_EQ_INT(v, (uint32_t)i);
        }
    }

    GW_CASE("信号量：初值与上限, 释放超过上限返回 FULL");
    {
        gw_sem_t *s = gw_sem_create("sem_t", 0u, 2u);
        GW_ASSERT(s != NULL);
        GW_ASSERT_EQ_INT(gw_sem_value(s), 0u);
        GW_ASSERT_EQ_INT(gw_sem_take(s, GW_NO_WAIT), GW_ERR_TIMEOUT);
        GW_ASSERT_EQ_INT(gw_sem_release(s), GW_OK);
        GW_ASSERT_EQ_INT(gw_sem_release(s), GW_OK);
        GW_ASSERT_EQ_INT(gw_sem_release(s), GW_ERR_FULL);
        GW_ASSERT_EQ_INT(gw_sem_value(s), 2u);
        GW_ASSERT_EQ_INT(gw_sem_take(s, 10), GW_OK);
        GW_ASSERT_EQ_INT(gw_sem_take(s, 10), GW_OK);
    }

    GW_CASE("事件标志组：与等待、或等待、超时、清除");
    {
        gw_thread_t *t;
        g_evt = gw_event_create("evt_t");
        GW_ASSERT(g_evt != NULL);
        g_flag_a = 0;
        g_flag_b = 0;
        t = gw_thread_create("evt", evt_thread, NULL, 4096u, 12u);
        GW_ASSERT(t != NULL);
        gw_port_delay_ms(50);
        GW_ASSERT_EQ_INT(gw_event_send(g_evt, 0x01u), 0x01u);
        gw_port_delay_ms(20);
        GW_ASSERT_EQ_INT(g_flag_a, 0);           /* 还差 bit1 */
        GW_ASSERT_EQ_INT(gw_event_send(g_evt, 0x02u), 0x03u);
        gw_port_delay_ms(50);
        GW_ASSERT_EQ_INT(g_flag_a, 1);           /* 与条件满足 */
        gw_port_delay_ms(400);
        GW_ASSERT_EQ_INT(g_flag_b, 1);           /* 或等待超时返回 0 */
        GW_ASSERT_EQ_INT(gw_event_clear(g_evt, 0xFFFFu), 0u);
        GW_ASSERT_EQ_INT(gw_event_recv(g_evt, 0x01u, false, GW_NO_WAIT), 0u);
    }

    GW_CASE("互斥锁：递归加锁/解锁与超时行为");
    {
        gw_mutex_t *m = gw_mutex_create("mtx_t", true);
        gw_mutex_pi_stats_t st;
        GW_ASSERT(m != NULL);
        GW_ASSERT_EQ_INT(gw_mutex_take(m, 100), GW_OK);
        GW_ASSERT_EQ_INT(gw_mutex_take(m, 100), GW_OK);      /* 递归 */
        GW_ASSERT_EQ_INT(gw_mutex_release(m), GW_OK);
        GW_ASSERT_EQ_INT(gw_mutex_release(m), GW_OK);
        GW_ASSERT_EQ_INT(gw_mutex_release(m), GW_ERR_STATE); /* 非持有者释放 */
        GW_ASSERT_EQ_INT(gw_mutex_pi_stats(m, &st), GW_OK);
        GW_ASSERT_EQ_INT(st.take_count, 2u);
        GW_ASSERT_EQ_INT(st.contend_count, 0u);   /* 同线程递归不算竞争 */
        GW_ASSERT_EQ_INT(st.inherit_count, 0u);
    }

    GW_CASE("看门狗：全部线程喂到则健康; 漏喂则检测到并要求复位");
    {
        gw_diag_t dg;
        GW_ASSERT_EQ_INT(gw_diag_init(&dg, 80u), GW_OK);
        gw_wdt_set_mask(GW_WDT_BIT_ACQ | GW_WDT_BIT_PARSE);
        gw_wdt_feed(GW_WDT_BIT_ACQ);
        gw_wdt_feed(GW_WDT_BIT_PARSE);
        GW_ASSERT(gw_wdt_check());
        GW_ASSERT(!dg.reset_pending);
        /* 只喂一个 -> 超时后应检测到未喂的位 */
        gw_port_delay_ms(100);
        gw_wdt_feed(GW_WDT_BIT_ACQ);
        GW_ASSERT(!gw_wdt_check());
        GW_ASSERT(dg.reset_pending);
        GW_ASSERT_EQ_INT(dg.reset_request_count, 1u);
        GW_ASSERT_EQ_INT(dg.last_missed_mask, GW_WDT_BIT_PARSE);
        /* 复位后重新喂满 -> 恢复健康 */
        gw_wdt_feed(GW_WDT_BIT_ACQ);
        gw_wdt_feed(GW_WDT_BIT_PARSE);
        GW_ASSERT(gw_wdt_check());
        GW_ASSERT_EQ_INT(dg.reset_request_count, 1u);
    }

    GW_CASE("线程栈高水位可测（>0 且 < 设计栈深）");
    {
        gw_thread_t *t = gw_thread_create("wa", heavy_thread, NULL, 8192u, 14u);
        int32_t used;
        GW_ASSERT(t != NULL);
        gw_port_delay_ms(120);
        used = gw_thread_stack_used(t);
        GW_ASSERT(used > 0);
        GW_ASSERT(used < 8192);
        GW_ASSERT_EQ_INT(gw_thread_stack_size(t), 8192);
        GW_ASSERT(!gw_thread_stack_overflow(t));
    }

    GW_CASE("栈溢出检测：实测消耗超过设计栈深时被检出并回调");
    {
        gw_diag_t    dg;
        gw_thread_t *t;
        int32_t      used;
        GW_ASSERT_EQ_INT(gw_diag_init(&dg, 100000u), GW_OK);
        /* 故意把设计栈深报成 512 字节, 而实际要 1KB 以上 -> 逻辑溢出 */
        t = gw_thread_create("tiny", heavy_thread, NULL, 512u, 14u);
        GW_ASSERT(t != NULL);
        gw_port_delay_ms(150);
        used = gw_thread_stack_used(t);
        GW_ASSERT(used > 512);                       /* 实测确实超了 */
        GW_ASSERT(gw_thread_stack_overflow(t));      /* 检测器报出来了 */
        GW_ASSERT(dg.overflow_count >= 1u);          /* 并回调到诊断服务 */
    }
}
