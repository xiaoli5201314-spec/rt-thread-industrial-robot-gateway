/*
 * port_rtthread.c - RT-Thread 目标板端口
 *
 * 本文件只在 RT-Thread 环境下编译（由 gateway/SConscript 收集），
 * 它把 port_rtos.h / port_hw.h 的抽象接口逐一映射到 RT-Thread API：
 *
 *   抽象接口                   RT-Thread 实现
 *   ------------------------------------------------------------------
 *   gw_thread_create      ->   rt_thread_create + rt_thread_startup
 *   gw_thread_stack_used  ->   由 struct rt_thread.sp 反推高水位
 *   gw_thread_stack_overflow -> rt_thread_stack_overflow_check（栈哨兵）
 *   gw_mutex_create       ->   rt_mutex_create(name, RT_IPC_FLAG_PRIO)
 *                              *** 这就是优先级继承的开关 ***
 *   gw_sem_create         ->   rt_sem_create(..., RT_IPC_FLAG_PRIO)
 *   gw_mq_create          ->   rt_mq_create / rt_mq_send / rt_mq_recv
 *   gw_event_create       ->   rt_event_create / rt_event_send / rt_event_recv
 *   gw_link (RS485)       ->   rt_device(serial) + DMA 接收 + rx_indicate
 *   gw_link (TCP)         ->   SAL: rt_socket/rt_connect/rt_send/rt_recv
 *   gw_link (CAN)         ->   rt_device(can) + rt_device_read(帧)
 *   gw_nv_*               ->   Flash 设备 + RT_DEVICE_CTRL_BLK_ERASE
 *   gw_wdt_*              ->   WDT 设备 + RT_DEVICE_CTRL_WDT_* 控制字
 *   gw_hw_get_faults      ->   硬件比较器 FAULT 引脚（GPIO 中断置位）
 *
 * 注意：本工程没有复制任何 RT-Thread 内核源码, 只通过公开 API 调用。
 */
#include "port_rtos.h"
#include "port_hw.h"
#include "gateway_config.h"
#include "gw_log.h"

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <string.h>

#ifdef RT_USING_SAL
#include <sys/socket.h>
#include <netdb.h>
#endif

/* ================================================================== */
/* 时间与日志出口                                                      */
/* ================================================================== */
void gw_port_init(void)
{
    /* 目标板无需额外初始化：时钟与设备由 BSP 完成 */
}

uint32_t gw_port_tick_ms(void)
{
    /* RT_TICK_PER_SECOND 默认 1000；若 BSP 改成 100, 这里自动换算 */
#if (RT_TICK_PER_SECOND == 1000)
    return (uint32_t)rt_tick_get();
#else
    return (uint32_t)(((uint64_t)rt_tick_get() * 1000u) / RT_TICK_PER_SECOND);
#endif
}

uint32_t gw_port_now_us(void)
{
    /* 微秒级时间戳：用高精度定时器（若 BSP 使能）, 否则用 tick 近似。
     * 帧同步状态机只需要"相对间隔", tick 精度不足时 T3.5 判定会放宽,
     * 因此工程上要求 BSP 打开 RT_USING_HWTIMER。 */
#ifdef RT_USING_HWTIMER
    {
        rt_device_t hwt = rt_device_find("hwtimer");
        if (hwt != RT_NULL) {
            rt_uint32_t cnt = 0u;
            if (rt_device_read(hwt, 0, &cnt, sizeof(cnt)) > 0) {
                return (uint32_t)cnt;
            }
        }
    }
#endif
    return gw_port_tick_ms() * 1000u;
}

void gw_port_delay_ms(uint32_t ms)
{
    rt_thread_mdelay((rt_int32_t)ms);
}

void gw_port_delay_us(uint32_t us)
{
    rt_hw_us_delay(us);
}

void gw_port_console_write(const char *s, size_t len)
{
    if ((s == NULL) || (len == 0u)) {
        return;
    }
    /* rt_kprintf 不支持定长输出, 这里按行切分打印 */
    rt_kprintf("%.*s", (int)len, s);
}

const char *gw_port_name(void)
{
    return "rt-thread";
}

bool gw_port_rt_sched_active(void)
{
    return true;      /* 目标板本来就是实时调度 */
}

const char *gw_port_sched_mode(void)
{
    return "RT-Thread 抢占式调度 (优先级 0..31, 数值越小越高)";
}

/* ================================================================== */
/* 线程                                                                */
/* ================================================================== */
struct gw_thread {
    char              name[RT_NAME_MAX];
    gw_thread_entry_t entry;
    void             *param;
    uint8_t           priority;
    uint32_t          stack_bytes;
    rt_thread_t       handle;
    rt_sem_t          exited;       /* 退出通知, 用于 join */
    struct gw_thread *next;
};

static struct gw_thread *s_threads = NULL;
static struct rt_mutex   s_reg_lock;
static bool              s_reg_lock_ready = false;

static void port_unused_result(rt_err_t rc)
{
    GW_UNUSED(rc);
}

static void gw_thread_trampoline(void *arg)
{
    struct gw_thread *t = (struct gw_thread *)arg;

    t->entry(t->param);
    if (t->exited != RT_NULL) {
        port_unused_result(rt_sem_release(t->exited));
    }
}

gw_thread_t *gw_thread_create(const char *name, gw_thread_entry_t entry,
                              void *param, uint32_t stack_bytes, uint8_t priority)
{
    struct gw_thread *t;

    if ((name == NULL) || (entry == NULL)) {
        return RT_NULL;
    }

    t = (struct gw_thread *)rt_malloc(sizeof(*t));
    if (t == RT_NULL) {
        return RT_NULL;
    }
    memset(t, 0, sizeof(*t));

    rt_strncpy(t->name, name, RT_NAME_MAX - 1);
    t->entry      = entry;
    t->param      = param;
    t->priority   = priority;
    t->stack_bytes = stack_bytes;

    /* RT-Thread 的栈大小按字节给出；内核会在栈底放置哨兵用于溢出检查 */
    t->handle = rt_thread_create(t->name, gw_thread_trampoline, t,
                                 (rt_uint32_t)stack_bytes,
                                 (rt_uint8_t)priority, 20);
    if (t->handle == RT_NULL) {
        rt_free(t);
        return RT_NULL;
    }
    t->exited = rt_sem_create(t->name, 0, RT_IPC_FLAG_PRIO);
    (void)rt_thread_startup(t->handle);

    /* 注册链表（用于诊断线程遍历栈水位） */
    if (!s_reg_lock_ready) {
        (void)rt_mutex_init(&s_reg_lock, "gwreg", RT_IPC_FLAG_PRIO);
        s_reg_lock_ready = true;
    }
    (void)rt_mutex_take(&s_reg_lock, RT_WAITING_FOREVER);
    t->next   = s_threads;
    s_threads = t;
    (void)rt_mutex_release(&s_reg_lock);

    return t;
}

int gw_thread_join(gw_thread_t *t, int32_t timeout_ms)
{
    rt_int32_t to;

    if ((t == NULL) || (t->exited == RT_NULL)) {
        return GW_ERR_PARAM;
    }
    to = (timeout_ms == GW_WAIT_FOREVER) ? RT_WAITING_FOREVER : (rt_int32_t)timeout_ms;
    return (rt_sem_take(t->exited, to) == RT_EOK) ? GW_OK : GW_ERR_TIMEOUT;
}

void gw_thread_yield(void)
{
    rt_thread_yield();
}

const char *gw_thread_self_name(void)
{
    rt_thread_t self = rt_thread_self();
    return (self != RT_NULL) ? self->name : "?";
}

uint8_t gw_thread_self_priority(void)
{
    rt_thread_t self = rt_thread_self();
    if (self == RT_NULL) {
        return 0xFFu;
    }
    /* 注意：RT-Thread 的 current_priority 在互斥量优先级继承发生时
     * 会被内核改写为"被提升后的优先级", 这正是我们要观测的量。 */
    return (uint8_t)self->current_priority;
}

int32_t gw_thread_stack_size(gw_thread_t *t)
{
    return (t != NULL) ? (int32_t)t->stack_bytes : -1;
}

int32_t gw_thread_stack_used(gw_thread_t *t)
{
    rt_uint32_t used;

    if ((t == NULL) || (t->handle == RT_NULL)) {
        return -1;
    }
    /* 与 msh 的 list_thread 同一算法：
     *   已用 = 栈总大小 - (当前 sp - 栈起始地址)                        */
    used = t->handle->stack_size -
           (rt_uint32_t)((rt_uint8_t *)t->handle->sp -
                         (rt_uint8_t *)t->handle->stack_addr);
    return (int32_t)used;
}

bool gw_thread_stack_overflow(gw_thread_t *t)
{
    if ((t == NULL) || (t->handle == RT_NULL)) {
        return false;
    }
    /* 需要 BSP 在 rtconfig.h 中使能 RT_USING_DEBUG 与栈溢出检查 */
    return (rt_thread_stack_overflow_check(t->handle) != RT_EOK) ? true : false;
}

static gw_stack_overflow_hook_t s_overflow_hook = NULL;

void gw_thread_set_overflow_hook(gw_stack_overflow_hook_t hook)
{
    s_overflow_hook = hook;
}

/* 由诊断线程调用；目标板上也可以通过定时器统一扫描 */
void gw_port_scan_stack_overflow(void)
{
    struct gw_thread *t = s_threads;

    while (t != NULL) {
        if (gw_thread_stack_overflow(t) && (s_overflow_hook != NULL)) {
            s_overflow_hook(t->name, (int32_t)t->stack_bytes);
        }
        t = t->next;
    }
}

/* ================================================================== */
/* 互斥锁：RT_IPC_FLAG_PRIO = 优先级继承                               */
/* ================================================================== */
struct gw_mutex {
    rt_mutex_t          handle;
    bool                pi;
    gw_mutex_pi_stats_t stats;
    struct gw_mutex    *next;
};

int gw_mutex_take(gw_mutex_t *m, int32_t timeout_ms);
int gw_mutex_release(gw_mutex_t *m);
int gw_mutex_pi_stats(gw_mutex_t *m, gw_mutex_pi_stats_t *out);

gw_mutex_t *gw_mutex_create(const char *name, bool priority_inherit)
{
    gw_mutex_t *m = (gw_mutex_t *)rt_malloc(sizeof(*m));

    if (m == NULL) {
        return NULL;
    }
    memset(m, 0, sizeof(*m));
    m->pi = priority_inherit;

    /*
     * RT_IPC_FLAG_PRIO：等待队列按优先级排序, 且持有者发生优先级继承
     *                  —— 这就是避免优先级反转的关键参数；
     * RT_IPC_FLAG_FIFO：先来先服务, 无优先级继承。
     * 工程上所有可能被"高优先级线程 + 低优先级持有者"同时访问的锁
     * 都必须用 PRIO。
     */
    m->handle = rt_mutex_create(name, priority_inherit ? RT_IPC_FLAG_PRIO
                                                       : RT_IPC_FLAG_FIFO);
    if (m->handle == RT_NULL) {
        rt_free(m);
        return NULL;
    }
    return m;
}

int gw_mutex_take(gw_mutex_t *m, int32_t timeout_ms)
{
    rt_int32_t to;
    rt_err_t   rc;

    if (m == NULL) {
        return GW_ERR_PARAM;
    }
    to = (timeout_ms == GW_WAIT_FOREVER) ? RT_WAITING_FOREVER
       : ((timeout_ms == GW_NO_WAIT) ? RT_WAITING_NO : (rt_int32_t)timeout_ms);

    /* 记录等待前的持有者优先级, 便于诊断输出 */
    if ((m->handle->owner != RT_NULL) && m->pi) {
        m->stats.last_owner_prio_base = (uint8_t)m->handle->owner->current_priority;
    }

    rc = rt_mutex_take(m->handle, to);
    if (rc != RT_EOK) {
        return (rc == -RT_ETIMEOUT) ? GW_ERR_TIMEOUT : GW_ERR;
    }

    m->stats.take_count++;
    if ((m->handle->hold == 1) && (m->handle->owner != RT_NULL) && m->pi) {
        /* hold == 1 表示不是递归进入：本次发生了真正的获取 */
        m->stats.last_owner_prio_boosted = (uint8_t)m->handle->owner->current_priority;
        if (m->stats.last_owner_prio_boosted < m->stats.last_owner_prio_base) {
            m->stats.inherit_count++;
            m->stats.last_waiter_prio = m->stats.last_owner_prio_boosted;
            GW_LOGI("PI", "mutex %s: owner prio %u -> %u (inherited)",
                    m->handle->parent.name,
                    (unsigned)m->stats.last_owner_prio_base,
                    (unsigned)m->stats.last_owner_prio_boosted);
        }
    }
    return GW_OK;
}

int gw_mutex_release(gw_mutex_t *m)
{
    if (m == NULL) {
        return GW_ERR_PARAM;
    }
    if (rt_mutex_release(m->handle) != RT_EOK) {
        return GW_ERR_STATE;
    }
    if (m->pi) {
        m->stats.restore_count++;
    }
    return GW_OK;
}

int gw_mutex_pi_stats(gw_mutex_t *m, gw_mutex_pi_stats_t *out)
{
    if ((m == NULL) || (out == NULL)) {
        return GW_ERR_PARAM;
    }
    *out = m->stats;
    return GW_OK;
}

/* ================================================================== */
/* 信号量                                                              */
/* ================================================================== */
struct gw_sem {
    rt_sem_t handle;
    uint32_t max_value;
};

gw_sem_t *gw_sem_create(const char *name, uint32_t initial, uint32_t max_value)
{
    gw_sem_t *s = (gw_sem_t *)rt_malloc(sizeof(*s));

    if (s == NULL) {
        return NULL;
    }
    s->max_value = (max_value == 0u) ? 1u : max_value;
    s->handle = rt_sem_create(name, (rt_uint32_t)initial, RT_IPC_FLAG_PRIO);
    if (s->handle == RT_NULL) {
        rt_free(s);
        return NULL;
    }
    return s;
}

int gw_sem_take(gw_sem_t *s, int32_t timeout_ms)
{
    rt_int32_t to;
    rt_err_t   rc;

    if (s == NULL) {
        return GW_ERR_PARAM;
    }
    to = (timeout_ms == GW_WAIT_FOREVER) ? RT_WAITING_FOREVER
       : ((timeout_ms == GW_NO_WAIT) ? RT_WAITING_NO : (rt_int32_t)timeout_ms);
    rc = rt_sem_take(s->handle, to);
    if (rc == RT_EOK) {
        return GW_OK;
    }
    return (rc == -RT_ETIMEOUT) ? GW_ERR_TIMEOUT : GW_ERR;
}

int gw_sem_release(gw_sem_t *s)
{
    if (s == NULL) {
        return GW_ERR_PARAM;
    }
    if ((uint32_t)s->handle->value >= s->max_value) {
        return GW_ERR_FULL;
    }
    return (rt_sem_release(s->handle) == RT_EOK) ? GW_OK : GW_ERR;
}

uint32_t gw_sem_value(gw_sem_t *s)
{
    return (s != NULL) ? (uint32_t)s->handle->value : 0xFFFFFFFFu;
}

/* ================================================================== */
/* 消息队列                                                            */
/* ================================================================== */
struct gw_mq {
    rt_mq_t  handle;
    size_t   msg_size;
    size_t   capacity;
};

gw_mq_t *gw_mq_create(const char *name, size_t msg_size, size_t max_msgs)
{
    gw_mq_t *q = (gw_mq_t *)rt_malloc(sizeof(*q));

    if ((q == NULL) || (msg_size == 0u) || (max_msgs == 0u)) {
        if (q != NULL) {
            rt_free(q);
        }
        return NULL;
    }
    q->msg_size = msg_size;
    q->capacity = max_msgs;
    q->handle   = rt_mq_create(name, (rt_uint32_t)msg_size,
                               (rt_uint32_t)max_msgs, RT_IPC_FLAG_PRIO);
    if (q->handle == RT_NULL) {
        rt_free(q);
        return NULL;
    }
    return q;
}

int gw_mq_send(gw_mq_t *q, const void *msg, int32_t timeout_ms)
{
    rt_int32_t to;
    rt_err_t   rc;

    if ((q == NULL) || (msg == NULL)) {
        return GW_ERR_PARAM;
    }
    to = (timeout_ms == GW_WAIT_FOREVER) ? RT_WAITING_FOREVER
       : ((timeout_ms == GW_NO_WAIT) ? RT_WAITING_NO : (rt_int32_t)timeout_ms);
    rc = rt_mq_send_wait(q->handle, msg, (rt_size_t)q->msg_size, to);
    if (rc == RT_EOK) {
        return GW_OK;
    }
    return (rc == -RT_EFULL) ? GW_ERR_FULL : GW_ERR_TIMEOUT;
}

int gw_mq_recv(gw_mq_t *q, void *msg, int32_t timeout_ms)
{
    rt_int32_t to;
    rt_err_t   rc;
    rt_size_t  got;

    if ((q == NULL) || (msg == NULL)) {
        return GW_ERR_PARAM;
    }
    to = (timeout_ms == GW_WAIT_FOREVER) ? RT_WAITING_FOREVER
       : ((timeout_ms == GW_NO_WAIT) ? RT_WAITING_NO : (rt_int32_t)timeout_ms);
    got = rt_mq_recv(q->handle, msg, (rt_size_t)q->msg_size, to);
    rc  = (rt_err_t)got;
    if (rc == RT_EOK) {
        return GW_OK;
    }
    if ((rc == -RT_ETIMEOUT) || (to == RT_WAITING_NO)) {
        return (to == RT_WAITING_NO) ? GW_ERR_EMPTY : GW_ERR_TIMEOUT;
    }
    return GW_ERR;
}

size_t gw_mq_count(gw_mq_t *q)
{
    return (q != NULL) ? (size_t)q->handle->entry : 0u;
}

size_t gw_mq_capacity(gw_mq_t *q)
{
    return (q != NULL) ? q->capacity : 0u;
}

/* ================================================================== */
/* 事件标志组                                                          */
/* ================================================================== */
struct gw_event {
    rt_event_t handle;
};

gw_event_t *gw_event_create(const char *name)
{
    gw_event_t *e = (gw_event_t *)rt_malloc(sizeof(*e));

    if (e == NULL) {
        return NULL;
    }
    e->handle = rt_event_create(name, RT_IPC_FLAG_PRIO);
    if (e->handle == RT_NULL) {
        rt_free(e);
        return NULL;
    }
    return e;
}

uint32_t gw_event_send(gw_event_t *e, uint32_t flags)
{
    if (e == NULL) {
        return 0u;
    }
    (void)rt_event_send(e->handle, (rt_uint32_t)flags);
    return (uint32_t)e->handle->set;
}

uint32_t gw_event_recv(gw_event_t *e, uint32_t interest, bool wait_all,
                       int32_t timeout_ms)
{
    rt_uint32_t recved = 0u;
    rt_int32_t  to;
    rt_uint8_t  opt;

    if ((e == NULL) || (interest == 0u)) {
        return 0u;
    }
    to  = (timeout_ms == GW_WAIT_FOREVER) ? RT_WAITING_FOREVER
        : ((timeout_ms == GW_NO_WAIT) ? RT_WAITING_NO : (rt_int32_t)timeout_ms);
    opt = (rt_uint8_t)(RT_EVENT_FLAG_OR | (wait_all ? RT_EVENT_FLAG_AND : 0u) |
                       ((timeout_ms == GW_NO_WAIT) ? RT_EVENT_FLAG_CLEAR : 0u));

    if (rt_event_recv(e->handle, (rt_uint32_t)interest, opt, to, &recved) == RT_EOK) {
        return (uint32_t)recved;
    }
    return 0u;
}

uint32_t gw_event_clear(gw_event_t *e, uint32_t flags)
{
    if (e == NULL) {
        return 0u;
    }
    (void)rt_event_control(e->handle, RT_IPC_CMD_RESET, NULL);
    return (uint32_t)e->handle->set & ~flags;
}

/* ================================================================== */
/* 看门狗（RT-Thread WDT 设备框架）                                    */
/* ================================================================== */
static rt_device_t s_wdt_dev = RT_NULL;
static uint32_t    s_wdt_mask = 0u;
static uint32_t    s_wdt_fed  = 0u;
static gw_wdt_reset_hook_t s_wdt_hook = NULL;

int gw_wdt_init(uint32_t timeout_ms)
{
    s_wdt_dev = rt_device_find("wdt");
    if (s_wdt_dev == RT_NULL) {
        GW_LOGW("WDT", "no wdt device found, watchdog disabled");
        return GW_ERR_NOT_FOUND;
    }
    (void)rt_device_open(s_wdt_dev, RT_DEVICE_OFLAG_RDWR);
    {
        rt_uint32_t to = timeout_ms;
        (void)rt_device_control(s_wdt_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &to);
        (void)rt_device_control(s_wdt_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    }
    s_wdt_mask = 0u;
    s_wdt_fed  = 0u;
    return GW_OK;
}

void gw_wdt_set_mask(uint32_t thread_mask)
{
    s_wdt_mask = thread_mask;
}

void gw_wdt_set_reset_hook(gw_wdt_reset_hook_t hook)
{
    s_wdt_hook = hook;
}

void gw_wdt_feed(uint32_t thread_bit)
{
    s_wdt_fed |= thread_bit;

    if (s_wdt_mask == 0u) {
        return;
    }
    if ((s_wdt_fed & s_wdt_mask) == s_wdt_mask) {
        s_wdt_fed = 0u;
        if (s_wdt_dev != RT_NULL) {
            (void)rt_device_control(s_wdt_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, RT_NULL);
        }
    }
}

bool gw_wdt_check(void)
{
    /* 目标板上真正的复位由硬件看门狗完成；这里只负责在"某个线程没喂狗"
     * 时上报, 让上层把上下文（哪个线程、当时的总线状态）写进日志,
     * 这样复位重启之后还能从 Flash 里的日志看出复位原因。 */
    if ((s_wdt_mask != 0u) && ((s_wdt_fed & s_wdt_mask) != s_wdt_mask)) {
        if (s_wdt_hook != NULL) {
            s_wdt_hook(s_wdt_mask & ~s_wdt_fed, "thread feed timeout");
        }
        return false;
    }
    return true;
}

/* ================================================================== */
/* 物理链路                                                            */
/* ================================================================== */
int gw_link_open(gw_link_t *l, const char *cfg);
int gw_link_send(gw_link_t *l, const uint8_t *data, size_t len, int32_t timeout_ms);
int gw_link_recv(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms);
void gw_link_close(gw_link_t *l);

/* ---------------- RS485 / 串口（DMA + 接收指示回调） ---------------- */
typedef struct {
    rt_device_t dev;
    rt_sem_t    rx_sem;      /* ISR/DMA 接收指示 -> 采集线程 */
    struct rt_semaphore rx_sem_obj;
    uint8_t     name[RT_NAME_MAX];
} rt_uart_priv_t;

static rt_err_t rt_uart_rx_indicate(rt_device_t dev, rt_size_t size)
{
    rt_uart_priv_t *p = (rt_uart_priv_t *)dev->user_data;

    GW_UNUSED(size);
    if (p != RT_NULL) {
        /* 中断上下文：只做"释放信号量"这一件事, 不做任何解析 */
        (void)rt_sem_release(&p->rx_sem_obj);
    }
    return RT_EOK;
}

static int rt_uart_open(gw_link_t *l, const char *cfg)
{
    rt_uart_priv_t *p = (rt_uart_priv_t *)l->priv;
    rt_uint16_t     flags = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX;

    if (cfg == NULL) {
        return GW_ERR_PARAM;
    }
    p->dev = rt_device_find(cfg);
    if (p->dev == RT_NULL) {
        GW_LOGE("LINK", "uart %s not found", cfg);
        return GW_ERR_NOT_FOUND;
    }

#ifdef RT_SERIAL_USING_DMA
    flags |= RT_DEVICE_FLAG_DMA_RX;      /* DMA 接收：高频报文不丢字节 */
#endif
    if (rt_device_open(p->dev, flags) != RT_EOK) {
        return GW_ERR_IO;
    }

    (void)rt_sem_init(&p->rx_sem_obj, "gwrx", 0, RT_IPC_FLAG_PRIO);
    p->dev->user_data = p;
    (void)rt_device_set_rx_indicate(p->dev, rt_uart_rx_indicate);

    /* 9600 8N1：与现场焊机/PLC 的默认参数一致 */
    {
        struct serial_configure sc = RT_SERIAL_CONFIG_DEFAULT;
        sc.baud_rate = BAUD_RATE_9600;
        sc.data_bits = DATA_BITS_8;
        sc.stop_bits = STOP_BITS_1;
        sc.parity    = PARITY_NONE;
        sc.bufsz     = 256;
        (void)rt_device_control(p->dev, RT_DEVICE_CTRL_CONFIG, &sc);
    }
    return GW_OK;
}

static int rt_uart_send(gw_link_t *l, const uint8_t *data, size_t len,
                        int32_t timeout_ms)
{
    rt_uart_priv_t *p = (rt_uart_priv_t *)l->priv;
    rt_size_t       n;

    GW_UNUSED(timeout_ms);
    if ((p == NULL) || (p->dev == RT_NULL)) {
        return GW_ERR_IO;
    }
    n = rt_device_write(p->dev, 0, data, len);
    return (n == len) ? (int)n : GW_ERR_IO;
}

static int rt_uart_recv(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms)
{
    rt_uart_priv_t *p = (rt_uart_priv_t *)l->priv;
    rt_size_t       n;

    if ((p == NULL) || (p->dev == RT_NULL)) {
        return GW_ERR_IO;
    }
    n = rt_device_read(p->dev, 0, buf, cap);
    if (n > 0u) {
        return (int)n;
    }
    /* 没有数据：等接收指示信号量（等价于 PC 端的 poll 等待） */
    {
        rt_int32_t to = (timeout_ms == GW_WAIT_FOREVER) ? RT_WAITING_FOREVER
                      : (rt_int32_t)timeout_ms;
        if (rt_sem_take(&p->rx_sem_obj, to) != RT_EOK) {
            return GW_ERR_TIMEOUT;
        }
    }
    n = rt_device_read(p->dev, 0, buf, cap);
    return (n > 0u) ? (int)n : GW_ERR_TIMEOUT;
}

static void rt_uart_close(gw_link_t *l)
{
    rt_uart_priv_t *p = (rt_uart_priv_t *)l->priv;
    if ((p != NULL) && (p->dev != RT_NULL)) {
        (void)rt_device_close(p->dev);
        p->dev = RT_NULL;
    }
}

/* ---------------- CAN（按帧读取） ---------------- */
typedef struct {
    rt_device_t dev;
} rt_can_priv_t;

static int rt_can_open(gw_link_t *l, const char *cfg)
{
    rt_can_priv_t *p = (rt_can_priv_t *)l->priv;

    p->dev = rt_device_find((cfg != NULL) ? cfg : "can1");
    if (p->dev == RT_NULL) {
        return GW_ERR_NOT_FOUND;
    }
#ifdef RT_CAN_USING_CANFD
    (void)rt_device_control(p->dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
#endif
    return (rt_device_open(p->dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX) == RT_EOK)
         ? GW_OK : GW_ERR_IO;
}

static int rt_can_send(gw_link_t *l, const uint8_t *data, size_t len,
                       int32_t timeout_ms)
{
    rt_can_priv_t *p = (rt_can_priv_t *)l->priv;
    struct rt_can_msg msg;

    GW_UNUSED(timeout_ms);
    if ((p == NULL) || (p->dev == RT_NULL) || (len > GW_CAN_MAX_DLC)) {
        return GW_ERR_PARAM;
    }
    rt_memset(&msg, 0, sizeof(msg));
    if (len >= 4u) {
        msg.id  = ((rt_uint32_t)data[0] << 24) | ((rt_uint32_t)data[1] << 16) |
                  ((rt_uint32_t)data[2] << 8) | (rt_uint32_t)data[3];
    }
    msg.ide = RT_CAN_STDID;
    msg.rtr = RT_CAN_DTR;
    msg.len = (rt_uint8_t)len;
    if (len > 4u) {
        rt_memcpy(msg.data, data + 4u, len - 4u);
    }
    return (rt_device_write(p->dev, 0, &msg, sizeof(msg)) == sizeof(msg))
         ? (int)len : GW_ERR_IO;
}

static int rt_can_recv(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms)
{
    rt_can_priv_t *p = (rt_can_priv_t *)l->priv;
    struct rt_can_msg msg;

    if ((p == NULL) || (p->dev == RT_NULL) || (cap < 4u)) {
        return GW_ERR_PARAM;
    }
    if (rt_device_read(p->dev, 0, &msg, sizeof(msg)) != sizeof(msg)) {
        /* 目标板上由 CAN 接收指示 + 信号量做阻塞等待, 这里简化为超时返回,
         * 由采集线程按 GW_POLL_PERIOD_MS 周期重试 */
        rt_thread_mdelay((timeout_ms == GW_WAIT_FOREVER) ? 1 : (rt_int32_t)timeout_ms);
        return GW_ERR_TIMEOUT;
    }
    buf[0] = (uint8_t)(msg.id >> 24);
    buf[1] = (uint8_t)(msg.id >> 16);
    buf[2] = (uint8_t)(msg.id >> 8);
    buf[3] = (uint8_t)msg.id;
    if ((cap >= 4u + msg.len) && (msg.len > 0u)) {
        rt_memcpy(buf + 4u, msg.data, msg.len);
    }
    return (int)(4u + msg.len);
}

static void rt_can_close(gw_link_t *l)
{
    rt_can_priv_t *p = (rt_can_priv_t *)l->priv;
    if ((p != NULL) && (p->dev != RT_NULL)) {
        (void)rt_device_close(p->dev);
    }
}

/* ---------------- TCP（SAL） ---------------- */
typedef struct {
    int sock;
} rt_tcp_priv_t;

static int rt_tcp_open(gw_link_t *l, const char *cfg)
{
    rt_tcp_priv_t *p = (rt_tcp_priv_t *)l->priv;
    char            host[32];
    int             port = 0;
    const char     *colon = rt_strchr(cfg, ':');

    GW_UNUSED(l);
    if (colon == RT_NULL) {
        return GW_ERR_PARAM;
    }
    {
        rt_size_t hl = (rt_size_t)(colon - cfg);
        if (hl >= sizeof(host)) {
            hl = sizeof(host) - 1u;
        }
        rt_memcpy(host, cfg, hl);
        host[hl] = '\0';
    }
    port = atoi(colon + 1);
    if ((port <= 0) || (port > 65535)) {
        return GW_ERR_PARAM;
    }

    p->sock = socket(AF_INET, SOCK_STREAM, 0);
    if (p->sock < 0) {
        return GW_ERR_IO;
    }
    {
        struct sockaddr_in sa;
        rt_memset(&sa, 0, sizeof(sa));
        sa.sin_family = AF_INET;
        sa.sin_port   = htons((uint16_t)port);
        sa.sin_addr.s_addr = inet_addr(host);
        if (connect(p->sock, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
            closesocket(p->sock);
            return GW_ERR_IO;
        }
    }
    return GW_OK;
}

static int rt_tcp_send(gw_link_t *l, const uint8_t *data, size_t len,
                       int32_t timeout_ms)
{
    rt_tcp_priv_t *p = (rt_tcp_priv_t *)l->priv;
    int            sent;

    GW_UNUSED(timeout_ms);
    sent = send(p->sock, data, (int)len, 0);
    return (sent == (int)len) ? sent : GW_ERR_IO;
}

static int rt_tcp_recv(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms)
{
    rt_tcp_priv_t *p = (rt_tcp_priv_t *)l->priv;
    struct timeval tv;
    int            n;

    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    (void)setsockopt(p->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    n = recv(p->sock, buf, (int)cap, 0);
    if (n > 0) {
        return n;
    }
    return (n == 0) ? GW_ERR_IO : GW_ERR_TIMEOUT;
}

static void rt_tcp_close(gw_link_t *l)
{
    rt_tcp_priv_t *p = (rt_tcp_priv_t *)l->priv;
    if ((p != NULL) && (p->sock >= 0)) {
        closesocket(p->sock);
        p->sock = -1;
    }
}

int gw_link_open(gw_link_t *l, const char *cfg)
{
    return ((l != NULL) && (l->open != NULL)) ? l->open(l, cfg) : GW_ERR_PARAM;
}

int gw_link_send(gw_link_t *l, const uint8_t *data, size_t len, int32_t timeout_ms)
{
    return ((l != NULL) && (l->send != NULL))
         ? l->send(l, data, len, timeout_ms) : GW_ERR_PARAM;
}

int gw_link_recv(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms)
{
    return ((l != NULL) && (l->recv != NULL))
         ? l->recv(l, buf, cap, timeout_ms) : GW_ERR_PARAM;
}

void gw_link_close(gw_link_t *l)
{
    if ((l != NULL) && (l->close != NULL)) {
        l->close(l);
    }
}

/* 链路工厂：目标板上由应用层指定设备名（uart2 / can1 / ip:port） */
gw_link_t *gw_link_rtthread_create(const char *kind, const char *cfg)
{
    gw_link_t *l = (gw_link_t *)rt_malloc(sizeof(*l));

    if (l == RT_NULL) {
        return NULL;
    }
    rt_memset(l, 0, sizeof(*l));

    if (rt_strcmp(kind, "uart") == 0) {
        rt_uart_priv_t *p = (rt_uart_priv_t *)rt_malloc(sizeof(*p));
        if (p == RT_NULL) {
            rt_free(l);
            return NULL;
        }
        rt_memset(p, 0, sizeof(*p));
        l->name  = "rt-uart";
        l->open  = rt_uart_open;
        l->send  = rt_uart_send;
        l->recv  = rt_uart_recv;
        l->close = rt_uart_close;
        l->priv  = p;
    } else if (rt_strcmp(kind, "can") == 0) {
        rt_can_priv_t *p = (rt_can_priv_t *)rt_malloc(sizeof(*p));
        if (p == RT_NULL) {
            rt_free(l);
            return NULL;
        }
        rt_memset(p, 0, sizeof(*p));
        l->name  = "rt-can";
        l->open  = rt_can_open;
        l->send  = rt_can_send;
        l->recv  = rt_can_recv;
        l->close = rt_can_close;
        l->priv  = p;
    } else {
        rt_tcp_priv_t *p = (rt_tcp_priv_t *)rt_malloc(sizeof(*p));
        if (p == RT_NULL) {
            rt_free(l);
            return NULL;
        }
        p->sock = -1;
        l->name  = "rt-tcp";
        l->open  = rt_tcp_open;
        l->send  = rt_tcp_send;
        l->recv  = rt_tcp_recv;
        l->close = rt_tcp_close;
        l->priv  = p;
    }
    GW_UNUSED(cfg);
    return l;
}

/* ================================================================== */
/* 非易失参数区（SPI NOR Flash, 按扇区擦写）                           */
/* ================================================================== */
static rt_device_t s_flash_dev = RT_NULL;

static rt_device_t nv_dev(void)
{
    if (s_flash_dev == RT_NULL) {
        s_flash_dev = rt_device_find("norflash0");
        if (s_flash_dev != RT_NULL) {
            (void)rt_device_open(s_flash_dev, RT_DEVICE_OFLAG_RDWR);
        }
    }
    return s_flash_dev;
}

int gw_nv_read(uint32_t offset, uint8_t *buf, size_t len)
{
    rt_device_t d = nv_dev();
    if (d == RT_NULL) {
        return GW_ERR_NOT_FOUND;
    }
    return (rt_device_read(d, (rt_off_t)offset, buf, len) == len) ? GW_OK : GW_ERR_IO;
}

int gw_nv_write(uint32_t offset, const uint8_t *buf, size_t len)
{
    rt_device_t d = nv_dev();
    if (d == RT_NULL) {
        return GW_ERR_NOT_FOUND;
    }
    return (rt_device_write(d, (rt_off_t)offset, buf, len) == len) ? GW_OK : GW_ERR_IO;
}

int gw_nv_erase(uint32_t offset, size_t len)
{
    rt_device_t d = nv_dev();
    struct rt_device_blk_geometry geo;

    if (d == RT_NULL) {
        return GW_ERR_NOT_FOUND;
    }
    rt_memset(&geo, 0, sizeof(geo));
    (void)rt_device_control(d, RT_DEVICE_CTRL_BLK_GETGEOME, &geo);

    /* 按扇区擦除；参数区从扇区边界开始对齐, 因此这里只需擦一整个扇区 */
    if (rt_device_control(d, RT_DEVICE_CTRL_BLK_ERASE, &offset) != RT_EOK) {
        return GW_ERR_IO;
    }
    GW_UNUSED(len);
    return GW_OK;
}

/* 目标板上没有"模拟掉电"的概念, 保留空实现以便上层代码无需条件编译 */
void gw_nv_simulate_torn_write(bool enable)
{
    GW_UNUSED(enable);
}

uint32_t gw_nv_get_erase_count(void)
{
    return 0u;      /* 目标板由 Flash 驱动统计, 这里不重复计数 */
}

uint32_t gw_nv_get_write_count(void)
{
    return 0u;
}

/* ================================================================== */
/* 硬件故障信号（GPIO 中断置位 + 应用层读取）                          */
/* ================================================================== */
static volatile uint32_t s_hw_faults = 0u;
static volatile bool     s_pwr_fail  = false;

uint32_t gw_hw_get_faults(void)
{
    return s_hw_faults;
}

void gw_hw_clear_faults(uint32_t mask)
{
    s_hw_faults &= ~mask;
}

void gw_hw_inject_fault(uint32_t mask)
{
    s_hw_faults |= mask;
}

bool gw_hw_power_fail_pending(void)
{
    return s_pwr_fail;
}

void gw_hw_inject_power_fail(bool pending)
{
    s_pwr_fail = pending;
}

/*
 * 引脚说明（与 docs/HARDWARE.md 的接口定义表对应）：
 *   PA0  电源 UVLO 比较器输出（下降沿中断 -> 掉电预警）
 *   PA1  RS485 收发器 FAULT（总线短路/过流）
 *   PA2  CAN 收发器 FAULT / 过流
 *   PA3  CAN 控制器 BUS-OFF 指示
 * 实际引脚号以控制板原理图为准。
 */
#define GW_PIN_PWR_FAIL    0
#define GW_PIN_RS485_FAULT 1
#define GW_PIN_CAN_FAULT   2
#define GW_PIN_CAN_BUSOFF  3

static void gw_pin_irq(void *args)
{
    rt_uint16_t pin = (rt_uint16_t)(rt_base_t)args;

    switch (pin) {
    case GW_PIN_PWR_FAIL:
        s_pwr_fail = true;
        s_hw_faults |= GW_HW_FAULT_PWR_UVLO;
        break;
    case GW_PIN_RS485_FAULT:
        s_hw_faults |= GW_HW_FAULT_RS485_SHORT;
        break;
    case GW_PIN_CAN_FAULT:
        s_hw_faults |= GW_HW_FAULT_CAN_OVERCUR;
        break;
    case GW_PIN_CAN_BUSOFF:
        s_hw_faults |= GW_HW_FAULT_CAN_BUS_OFF;
        break;
    default:
        break;
    }
}

int gw_hw_fault_init(void)
{
    rt_pin_mode(GW_PIN_PWR_FAIL, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(GW_PIN_RS485_FAULT, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(GW_PIN_CAN_FAULT, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(GW_PIN_CAN_BUSOFF, PIN_MODE_INPUT_PULLUP);

    (void)rt_pin_attach_irq(GW_PIN_PWR_FAIL, PIN_IRQ_MODE_FALLING,
                            gw_pin_irq, (void *)(rt_base_t)GW_PIN_PWR_FAIL);
    (void)rt_pin_attach_irq(GW_PIN_RS485_FAULT, PIN_IRQ_MODE_FALLING,
                            gw_pin_irq, (void *)(rt_base_t)GW_PIN_RS485_FAULT);
    (void)rt_pin_attach_irq(GW_PIN_CAN_FAULT, PIN_IRQ_MODE_FALLING,
                            gw_pin_irq, (void *)(rt_base_t)GW_PIN_CAN_FAULT);
    (void)rt_pin_attach_irq(GW_PIN_CAN_BUSOFF, PIN_IRQ_MODE_RISING,
                            gw_pin_irq, (void *)(rt_base_t)GW_PIN_CAN_BUSOFF);

    (void)rt_pin_irq_enable(GW_PIN_PWR_FAIL, PIN_IRQ_ENABLE);
    (void)rt_pin_irq_enable(GW_PIN_RS485_FAULT, PIN_IRQ_ENABLE);
    (void)rt_pin_irq_enable(GW_PIN_CAN_FAULT, PIN_IRQ_ENABLE);
    (void)rt_pin_irq_enable(GW_PIN_CAN_BUSOFF, PIN_IRQ_ENABLE);
    return GW_OK;
}
