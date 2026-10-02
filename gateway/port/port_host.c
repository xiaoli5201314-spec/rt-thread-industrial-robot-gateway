/*
 * port_host.c - PC 仿真端口（pthread 实现）
 *
 * 用途：在没有目标板的情况下, 让整套网关逻辑跑在真实抢占式线程上,
 * 这样单元测试里断言的"互斥锁阻塞"、"信号量唤醒"、"消息队列满载"
 * 都是真实发生的, 而不是打桩返回。
 *
 * 与目标板的差异（如实记录, 也在 README 中说明）：
 *   1) 优先级：逻辑优先级仍沿用 RT-Thread 语义（0 最高）。若进程有
 *      CAP_SYS_NICE（root）且设置环境变量 GW_RT_SCHED=1, 端口会把逻辑
 *      优先级映射到 SCHED_FIFO 的真实实时优先级, 此时"优先级继承"
 *      是操作系统真实行为; 否则退化为 SCHED_OTHER(CFS), 依赖本层
 *      逻辑优先级记账来观测继承过程。
 *   2) 栈：目标板用 rt_thread_create 的静态/堆栈区, 溢出由 MPU 或
 *      栈哨兵检查; PC 端口自己 mmap 一块区域并整体涂色 0xA5, 用
 *      "最低被写地址"反推高水位, 用栈底 64 字节红区判断溢出。
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "port_rtos.h"
#include "gw_log.h"

#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sched.h>

/* ================================================================== */
/* 基础：时间与日志出口                                                */
/* ================================================================== */
static bool s_port_ready = false;

static void port_ensure_ready(void)
{
    if (!s_port_ready) {
        gw_port_init();
    }
}

void gw_port_init(void)
{
    if (s_port_ready) {
        return;
    }
    s_port_ready = true;
}

uint32_t gw_port_tick_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(((uint64_t)ts.tv_sec * 1000u) + ((uint64_t)ts.tv_nsec / 1000000u));
}

uint32_t gw_port_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(((uint64_t)ts.tv_sec * 1000000u) + ((uint64_t)ts.tv_nsec / 1000u));
}

void gw_port_delay_ms(uint32_t ms)
{
    struct timespec ts;
    ts.tv_sec  = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)((ms % 1000u) * 1000000u);
    while ((nanosleep(&ts, &ts) == -1) && (errno == EINTR)) {
        /* 被信号打断后继续睡剩余时间 */
    }
}

void gw_port_delay_us(uint32_t us)
{
    struct timespec ts;
    ts.tv_sec  = (time_t)(us / 1000000u);
    ts.tv_nsec = (long)((us % 1000000u) * 1000u);
    while ((nanosleep(&ts, &ts) == -1) && (errno == EINTR)) {
    }
}

void gw_port_console_write(const char *s, size_t len)
{
    ssize_t rc;
    if ((s == NULL) || (len == 0u)) {
        return;
    }
    /* 一次 write 系统调用：不经过 stdio 缓冲, 多线程下不会交错 */
    rc = write(STDERR_FILENO, s, len);
    GW_UNUSED(rc);
}

const char *gw_port_name(void)
{
    return "host-pthread";
}

/* ================================================================== */
/* 线程                                                                */
/* ================================================================== */
#define GW_STACK_PAINT        0xA5u
#define GW_STACK_GUARD_BYTES  64u     /* 整块栈区最底部的硬红区            */
#define GW_STACK_TCB_RESERVE  2048u   /* 栈顶预留：glibc TCB + 静态 TLS。
                                       * 必须**小于**实测启动足迹(约3.8KB),
                                       * 这样 base_low 才能落在涂色区内, 由
                                       * "base_low - 当前最低已写地址"自动把
                                       * 启动帧开销扣掉；预留过大反而会让涂色
                                       * 区落在启动足迹之下, 高水位恒为 0。   */
#define GW_STACK_SLACK_BELOW  8192u   /* 逻辑预算之外的越界余量            */
#define GW_STACK_MIN_TOTAL    32768u  /* 保证 pthread_attr_setstack 可用   */

struct gw_thread {
    char              name[16];
    gw_thread_entry_t entry;
    void             *param;
    uint8_t           priority;      /* 逻辑优先级, 0 最高               */
    uint8_t           cur_priority;  /* 可能被优先级继承临时提升         */
    uint32_t          stack_bytes;   /* 逻辑栈深（对外的"设计值"）       */
    uint8_t          *alloc;         /* 整块分配的首地址                 */
    uint8_t          *guard;         /* 红区起始（逻辑栈底以下 64B）     */
    uint8_t          *region_lo;     /* 逻辑栈区低地址                   */
    uint8_t          *region_hi;     /* 逻辑栈区高地址                   */
    uint8_t          *base_low;      /* 线程刚进入时的栈顶参考           */
    uint8_t          *final_low;     /* 线程结束时/查询时的最低被写地址  */
    uint8_t          *last_low;      /* 高水位扫描缓存（单调下降）       */
    pthread_t         tid;
    bool              started;
    bool              finished;
    bool              rt_sched;
    bool              overflow_reported;
    struct gw_thread *next;
};

static struct gw_thread *s_threads = NULL;
static pthread_mutex_t   s_threads_lock = PTHREAD_MUTEX_INITIALIZER;

/*
 * 主线程也要是一个"一等公民"线程对象。
 * 否则主线程调用 gw_mutex_take 时 tls_self 为 NULL, 递归加锁判定
 * （owner == self）永远不成立, 会退化成"把自己当别的线程", 递归语义
 * 与错误检测都会失效。
 */
static struct gw_thread s_main_thread = {
    .name       = "main",
    .entry      = NULL,
    .param      = NULL,
    .priority   = 10u,
    .cur_priority = 10u,
    .stack_bytes  = 0u,
    .alloc      = NULL,
    .guard      = NULL,
    .region_lo  = NULL,
    .region_hi  = NULL,
    .base_low   = NULL,
    .final_low  = NULL,
    .last_low   = NULL,
    .tid        = (pthread_t)0,
    .started    = true,
    .finished   = false,
    .rt_sched   = false,
    .overflow_reported = false,
    .next       = NULL
};
static __thread struct gw_thread *tls_self = &s_main_thread;

static gw_stack_overflow_hook_t s_overflow_hook = NULL;
static bool s_rtsched_requested = false;
static bool s_rtsched_active    = false;
static bool s_rtsched_checked   = false;

/* 逻辑优先级(0 最高) -> POSIX 实时优先级(数值越大越高) */
static int port_prio_to_posix(uint8_t logical)
{
    int p = 90 - (int)logical;
    if (p < 1) {
        p = 1;
    }
    if (p > 99) {
        p = 99;
    }
    return p;
}

bool gw_port_rt_sched_active(void)
{
    return s_rtsched_active;
}

const char *gw_port_sched_mode(void)
{
    return s_rtsched_active ? "SCHED_FIFO" : "SCHED_OTHER(CFS)";
}

/* 扫描涂色区, 返回最低的"被写过"的地址（未被写过的是 0xA5）。
 * 高水位只会单调下降, 因此只需从上次结果向下扩展扫描窗口,
 * 稳态下每次调用只比较 1 个字节, 不会因为反复扫 30KB 而吃掉 CPU。 */
static uint8_t *port_scan_low_watermark(struct gw_thread *t)
{
    uint8_t *p;

    if ((t->last_low == NULL) || (t->last_low > t->region_hi)) {
        t->last_low = t->region_hi;
    }
    for (p = t->region_lo; p < t->last_low; p++) {
        if (*p != (uint8_t)GW_STACK_PAINT) {
            t->last_low = p;
            return p;
        }
    }
    return t->last_low;
}

static void port_thread_register(struct gw_thread *t)
{
    pthread_mutex_lock(&s_threads_lock);
    t->next  = s_threads;
    s_threads = t;
    pthread_mutex_unlock(&s_threads_lock);
}

static void *port_thread_trampoline(void *arg)
{
    struct gw_thread *t = (struct gw_thread *)arg;
    struct sched_param sp;
    volatile uint8_t   stack_marker = 0u;   /* 取地址 = 线程入口处的栈顶 */

    tls_self = t;

    /* 尝试切换到实时调度策略：只有 root（CAP_SYS_NICE）能成功 */
    if (s_rtsched_requested) {
        memset(&sp, 0, sizeof(sp));
        sp.sched_priority = port_prio_to_posix(t->priority);
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0) {
            t->rt_sched = true;
            /*
             * 关键：把实时线程绑到同一个 CPU 上。
             * 多核主机上"优先级"只在同一颗核内才有意义 —— 不绑核的话,
             * 低优先级线程会在另一颗核上照跑, 优先级反转根本不会发生,
             * 也就无法验证优先级继承。绑核后 PC 仿真的调度行为与
             * 单核 MCU 上的 RT-Thread 一致。
             */
            {
                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                CPU_SET(0, &cpuset);
                (void)sched_setaffinity(0, sizeof(cpuset), &cpuset);
            }
            if (!s_rtsched_checked) {
                s_rtsched_checked = true;
                s_rtsched_active  = true;
                GW_LOGI("PORT", "SCHED_FIFO enabled, logical prio %u -> posix %d, "
                                "pinned to CPU0",
                        (unsigned)t->priority, sp.sched_priority);
            }
        } else if (!s_rtsched_checked) {
            s_rtsched_checked = true;
            s_rtsched_active  = false;
            GW_LOGW("PORT", "SCHED_FIFO unavailable (%s), fallback to CFS; "
                            "logical priorities are tracked by the port layer",
                    strerror(errno));
        }
    } else if (!s_rtsched_checked) {
        s_rtsched_checked = true;
        s_rtsched_active  = false;
    }

#if defined(__linux__)
    {
        char nm[16];
        snprintf(nm, sizeof(nm), "%s", t->name);
        (void)pthread_setname_np(pthread_self(), nm);
    }
#endif

    /*
     * 基线水位：用线程入口处局部变量的地址作为"栈顶"参考点。
     *
     * 为什么不直接用"最低已写地址"当初值：glibc 在进入用户线程函数之前
     * 已经深挖过一次栈（TCB/静态 TLS/start_thread 启动帧，实测约 2KB），
     * 因此"进入时的最低已写地址"比真正的栈顶低得多, 会让高水位系统性
     * 偏小甚至恒为 0。取局部变量地址则等价于"从线程函数入口开始计量",
     * 这正是我们给线程分配栈深时要对比的口径。
     */
    t->base_low  = (uint8_t *)(uintptr_t)&stack_marker;
    if (t->base_low < t->region_lo) {
        t->base_low = t->region_lo;
    }
    if (t->base_low > t->region_hi) {
        t->base_low = t->region_hi;
    }

    /*
     * 抹掉"启动尖峰"：glibc 在调用用户线程函数之前会深挖一次栈
     * （TCB/静态 TLS/start_thread）, 最深处比本函数入口还低约 2KB。
     * 涂色高水位是"历史最深"语义, 若不擦除, 那个尖峰会变成所有线程
     * 共同的测量地板（实测恰好 2083 字节, 与线程实际负载无关）。
     * 因此在入口处把 base_low 以下的涂色区重新涂一遍, 让"历史最深"
     * 从线程函数入口重新起算 —— 这正是给线程定栈深时要对比的口径。
     */
    {
        uint8_t *safe = t->base_low - 256u;   /* 保住 128B 红区与本栈帧 */
        if (safe > t->region_lo) {
            memset(t->region_lo, GW_STACK_PAINT, (size_t)(safe - t->region_lo));
        }
        memset(t->guard, GW_STACK_PAINT, GW_STACK_GUARD_BYTES);
        t->last_low = t->base_low;
    }

    t->entry(t->param);

    t->final_low = port_scan_low_watermark(t);

    pthread_mutex_lock(&s_threads_lock);
    t->finished = true;
    pthread_mutex_unlock(&s_threads_lock);
    return NULL;
}

gw_thread_t *gw_thread_create(const char *name, gw_thread_entry_t entry,
                              void *param, uint32_t stack_bytes, uint8_t priority)
{
    struct gw_thread *t;
    pthread_attr_t    attr;
    size_t            total;
    const char       *env;

    port_ensure_ready();

    if ((entry == NULL) || (stack_bytes < 256u) || (name == NULL)) {
        return NULL;
    }

    env = getenv("GW_RT_SCHED");
    s_rtsched_requested = (env != NULL) && (env[0] == '1');

    t = (struct gw_thread *)calloc(1, sizeof(*t));
    if (t == NULL) {
        return NULL;
    }

    /*
     * 栈布局（这里的细节是实测踩出来的, 见 README"栈深标定"一节）：
     *
     *   +--------------------------+ <- mem + total  （glibc 在此放 TCB）
     *   |  TCB/TLS/启动帧 8KB 预留  |
     *   +--------------------------+ <- region_hi（涂色区上界）
     *   |      涂色区 0xA5          |
     *   |  线程运行期 SP 在区内下移  |
     *   +--------------------------+ <- region_lo
     *   |  余量 slack               |
     *   +--------------------------+ <- guard（64B 硬红区）
     *   |      <- 分配起点 mem      |
     *   +--------------------------+
     *
     * 高水位按"进入线程时的最低已写地址(base_low) - 当前最低已写地址"计算，
     * 这样自动扣掉了 TCB/启动帧那 3~4KB, 得到的才是线程自身代码路径的真实
     * 栈消耗。溢出判定 = 该消耗超过设计栈深, 或最底部硬红区被写坏。
     */
    {
        uint32_t slack = GW_STACK_SLACK_BELOW;
        if ((GW_STACK_TCB_RESERVE + stack_bytes + slack) < GW_STACK_MIN_TOTAL) {
            slack = GW_STACK_MIN_TOTAL - GW_STACK_TCB_RESERVE - stack_bytes;
        }
        total = (size_t)GW_STACK_TCB_RESERVE + stack_bytes + slack;
    }
    t->alloc = (uint8_t *)aligned_alloc(64u, (total + 63u) & ~(size_t)63u);
    if (t->alloc == NULL) {
        free(t);
        return NULL;
    }
    memset(t->alloc, GW_STACK_PAINT, total);

    t->guard       = t->alloc;
    t->region_lo   = t->alloc + GW_STACK_GUARD_BYTES;
    t->region_hi   = t->alloc + total - GW_STACK_TCB_RESERVE;
    t->stack_bytes = stack_bytes;
    t->base_low    = t->region_hi;
    t->final_low   = t->region_hi;
    t->last_low    = t->region_hi;

    snprintf(t->name, sizeof(t->name), "%s", name);
    t->entry        = entry;
    t->param        = param;
    t->priority     = priority;
    t->cur_priority = priority;

    if (pthread_attr_init(&attr) != 0) {
        free(t->alloc);
        free(t);
        return NULL;
    }
    (void)pthread_attr_setstack(&attr, t->alloc, total);

    if (pthread_create(&t->tid, &attr, port_thread_trampoline, t) != 0) {
        pthread_attr_destroy(&attr);
        free(t->alloc);
        free(t);
        return NULL;
    }
    pthread_attr_destroy(&attr);

    t->started = true;
    port_thread_register(t);
    return t;
}

int gw_thread_join(gw_thread_t *t, int32_t timeout_ms)
{
    struct timespec ts;
    int waited_ms = 0;

    if (t == NULL) {
        return GW_ERR_PARAM;
    }

    for (;;) {
        pthread_mutex_lock(&s_threads_lock);
        if (t->finished) {
            pthread_mutex_unlock(&s_threads_lock);
            break;
        }
        pthread_mutex_unlock(&s_threads_lock);

        if (timeout_ms == 0) {
            return GW_ERR_TIMEOUT;
        }
        gw_port_delay_ms(1);
        waited_ms++;
        if ((timeout_ms > 0) && (waited_ms >= timeout_ms)) {
            return GW_ERR_TIMEOUT;
        }
    }

    /* 回收线程资源（t 本身留给调用者继续查询栈水位, 进程退出时统一释放）*/
    ts.tv_sec  = 0;
    ts.tv_nsec = 1000000;   /* 1ms, 给 trampoline 收尾 */
    nanosleep(&ts, NULL);
    return GW_OK;
}

void gw_thread_yield(void)
{
    sched_yield();
}

const char *gw_thread_self_name(void)
{
    return (tls_self != NULL) ? tls_self->name : "main";
}

uint8_t gw_thread_self_priority(void)
{
    return (tls_self != NULL) ? tls_self->cur_priority : 0u;
}

int32_t gw_thread_stack_size(gw_thread_t *t)
{
    return (t != NULL) ? (int32_t)t->stack_bytes : -1;
}

int32_t gw_thread_stack_used(gw_thread_t *t)
{
    uint8_t *low;
    size_t   used;

    if (t == NULL) {
        return -1;
    }

    low = (t->finished && (t->final_low != NULL)) ? t->final_low
                                                  : port_scan_low_watermark(t);
    if (low >= t->base_low) {
        return 0;
    }
    used = (size_t)(t->base_low - low);
    return (int32_t)used;
}

bool gw_thread_stack_overflow(gw_thread_t *t)
{
    uint8_t *p;
    bool     hard = false;
    bool     logic = false;

    if ((t == NULL) || (t->guard == NULL)) {
        return false;
    }

    /* 1) 硬红区：连整块分配的底部都被写坏, 说明远超预算 */
    for (p = t->guard; p < t->guard + GW_STACK_GUARD_BYTES; p++) {
        if (*p != (uint8_t)GW_STACK_PAINT) {
            hard = true;
            break;
        }
    }

    /* 2) 逻辑溢出：实测高水位超过该线程的设计栈深 */
    if (gw_thread_stack_used(t) > (int32_t)t->stack_bytes) {
        logic = true;
    }

    if (!hard && !logic) {
        return false;
    }
    if (!t->overflow_reported) {
        t->overflow_reported = true;
        if (s_overflow_hook != NULL) {
            s_overflow_hook(t->name, (int32_t)t->stack_bytes);
        }
    }
    return true;
}

void gw_thread_set_overflow_hook(gw_stack_overflow_hook_t hook)
{
    s_overflow_hook = hook;
}

/* ================================================================== */
/* 互斥锁 + 优先级继承                                                 */
/* ================================================================== */
struct gw_mutex {
    char                name[16];
    bool                pi;             /* 是否启用优先级继承           */
    pthread_mutex_t     h;
    pthread_mutex_t     lock;           /* 保护下列元数据               */
    struct gw_thread   *owner;
    uint32_t            recursion;
    uint32_t            hold_start_us;
    uint32_t            nwaiters;
    bool                boosted;
    uint8_t             boost_saved_prio;
    gw_mutex_pi_stats_t stats;
};

gw_mutex_t *gw_mutex_create(const char *name, bool priority_inherit)
{
    struct gw_mutex *m;
    pthread_mutexattr_t attr;

    port_ensure_ready();

    m = (struct gw_mutex *)calloc(1, sizeof(*m));
    if (m == NULL) {
        return NULL;
    }
    snprintf(m->name, sizeof(m->name), "%s", (name != NULL) ? name : "mtx");
    m->pi = priority_inherit;

    if (pthread_mutexattr_init(&attr) != 0) {
        free(m);
        return NULL;
    }
    (void)pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    /* RT-Thread 的 RT_IPC_FLAG_PRIO 互斥量等价物：
     * POSIX 优先级继承协议, 持有者被临时提升到等待者的优先级。 */
    (void)pthread_mutexattr_setprotocol(&attr,
            priority_inherit ? PTHREAD_PRIO_INHERIT : PTHREAD_PRIO_NONE);

    if (pthread_mutex_init(&m->h, &attr) != 0) {
        pthread_mutexattr_destroy(&attr);
        free(m);
        return NULL;
    }
    pthread_mutexattr_destroy(&attr);
    pthread_mutex_init(&m->lock, NULL);
    return m;
}

/* 在持有 m->lock 的前提下记录一次优先级提升 */
static void port_mutex_boost_locked(struct gw_mutex *m, struct gw_thread *waiter)
{
    struct gw_thread *owner = m->owner;

    if ((owner == NULL) || (waiter == NULL)) {
        return;
    }
    if (waiter->cur_priority >= owner->cur_priority) {
        return;   /* 等待者优先级并不更高, 无需继承 */
    }

    m->stats.last_owner_prio_base    = owner->priority;
    m->stats.last_owner_prio_boosted = waiter->cur_priority;
    m->stats.last_waiter_prio        = waiter->cur_priority;
    m->stats.inherit_count++;

    if (!m->boosted) {
        m->boost_saved_prio = owner->cur_priority;
        m->boosted = true;
    }
    owner->cur_priority = waiter->cur_priority;

    GW_LOGI("PI", "mutex=%s owner=%s prio %u -> %u (waiter=%s %u) [inherit #%u]",
            m->name, owner->name,
            (unsigned)m->stats.last_owner_prio_base,
            (unsigned)owner->cur_priority,
            waiter->name, (unsigned)waiter->cur_priority,
            (unsigned)m->stats.inherit_count);

    /* 若已切到 SCHED_FIFO, 同步刷新真实调度优先级, 让继承在 OS 层生效 */
    if (owner->rt_sched) {
        struct sched_param sp;
        memset(&sp, 0, sizeof(sp));
        sp.sched_priority = port_prio_to_posix(owner->cur_priority);
        (void)pthread_setschedparam(owner->tid, SCHED_FIFO, &sp);
    }
}

/* 在持有 m->lock 的前提下撤销提升 */
static void port_mutex_restore_locked(struct gw_mutex *m)
{
    struct gw_thread *owner = m->owner;

    if (!m->boosted || (owner == NULL)) {
        return;
    }
    owner->cur_priority = m->boost_saved_prio;
    m->boosted = false;
    m->stats.restore_count++;

    GW_LOGI("PI", "mutex=%s owner=%s restored prio -> %u [restore #%u]",
            m->name, owner->name, (unsigned)owner->cur_priority,
            (unsigned)m->stats.restore_count);

    if (owner->rt_sched) {
        struct sched_param sp;
        memset(&sp, 0, sizeof(sp));
        sp.sched_priority = port_prio_to_posix(owner->cur_priority);
        (void)pthread_setschedparam(owner->tid, SCHED_FIFO, &sp);
    }
}

int gw_mutex_take(gw_mutex_t *m, int32_t timeout_ms)
{
    struct gw_thread *self = tls_self;
    bool   will_block;
    int    rc = 0;

    if (m == NULL) {
        return GW_ERR_PARAM;
    }

    pthread_mutex_lock(&m->lock);

    if ((m->owner != NULL) && (m->owner == self)) {
        m->recursion++;
        m->stats.take_count++;
        pthread_mutex_unlock(&m->lock);
        return GW_OK;   /* 递归加锁, 不阻塞 */
    }

    will_block = (m->owner != NULL);
    if (will_block) {
        m->stats.contend_count++;
        m->nwaiters++;
        if (m->pi) {
            port_mutex_boost_locked(m, self);
        }
    }
    pthread_mutex_unlock(&m->lock);

    if (timeout_ms == GW_WAIT_FOREVER) {
        rc = pthread_mutex_lock(&m->h);
    } else if (timeout_ms == GW_NO_WAIT) {
        rc = pthread_mutex_trylock(&m->h);
    } else {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += (time_t)(timeout_ms / 1000);
        ts.tv_nsec += (long)((timeout_ms % 1000) * 1000000);
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec  += 1;
            ts.tv_nsec -= 1000000000L;
        }
        rc = pthread_mutex_timedlock(&m->h, &ts);
    }

    pthread_mutex_lock(&m->lock);
    if (rc != 0) {
        if (will_block) {
            if (m->nwaiters > 0u) {
                m->nwaiters--;
            }
            /* 等待失败：若已无人竞争则撤销提升 */
            if (m->pi && (m->nwaiters == 0u)) {
                port_mutex_restore_locked(m);
            }
        }
        pthread_mutex_unlock(&m->lock);
        return GW_ERR_TIMEOUT;
    }

    if (will_block && (m->nwaiters > 0u)) {
        m->nwaiters--;
    }
    m->owner          = self;
    m->recursion      = 1;
    m->hold_start_us  = gw_port_now_us();
    m->stats.take_count++;
    pthread_mutex_unlock(&m->lock);
    return GW_OK;
}

int gw_mutex_release(gw_mutex_t *m)
{
    if (m == NULL) {
        return GW_ERR_PARAM;
    }

    pthread_mutex_lock(&m->lock);
    if (m->owner != tls_self) {
        pthread_mutex_unlock(&m->lock);
        return GW_ERR_STATE;   /* 非持有者释放 / 重复释放 */
    }

    if (m->recursion > 0u) {
        m->recursion--;
    }
    if (m->recursion > 0u) {
        pthread_mutex_unlock(&m->lock);
        return GW_OK;
    }

    {
        uint32_t held = gw_port_now_us() - m->hold_start_us;
        if (held > m->stats.max_hold_us) {
            m->stats.max_hold_us = held;
        }
    }

    m->owner = NULL;
    if (m->pi) {
        /* 恢复的是"上一个持有者"（也就是刚释放的线程）的优先级 */
        if (m->boosted && (tls_self != NULL)) {
            struct gw_thread *owner = tls_self;
            owner->cur_priority = m->boost_saved_prio;
            m->boosted = false;
            m->stats.restore_count++;
            GW_LOGI("PI", "mutex=%s owner=%s restored prio -> %u [restore #%u]",
                    m->name, owner->name, (unsigned)owner->cur_priority,
                    (unsigned)m->stats.restore_count);
            if (owner->rt_sched) {
                struct sched_param sp;
                memset(&sp, 0, sizeof(sp));
                sp.sched_priority = port_prio_to_posix(owner->cur_priority);
                (void)pthread_setschedparam(owner->tid, SCHED_FIFO, &sp);
            }
        }
    }
    pthread_mutex_unlock(&m->lock);

    pthread_mutex_unlock(&m->h);
    return GW_OK;
}

int gw_mutex_pi_stats(gw_mutex_t *m, gw_mutex_pi_stats_t *out)
{
    if ((m == NULL) || (out == NULL)) {
        return GW_ERR_PARAM;
    }
    pthread_mutex_lock(&m->lock);
    *out = m->stats;
    pthread_mutex_unlock(&m->lock);
    return GW_OK;
}

/* ================================================================== */
/* 信号量                                                              */
/* ================================================================== */
struct gw_sem {
    char    name[16];
    sem_t   h;
    uint32_t max_value;
    uint32_t initial;
};

gw_sem_t *gw_sem_create(const char *name, uint32_t initial, uint32_t max_value)
{
    struct gw_sem *s;

    port_ensure_ready();
    s = (struct gw_sem *)calloc(1, sizeof(*s));
    if (s == NULL) {
        return NULL;
    }
    snprintf(s->name, sizeof(s->name), "%s", (name != NULL) ? name : "sem");
    s->max_value = (max_value == 0u) ? 1u : max_value;
    s->initial   = initial;
    if (sem_init(&s->h, 0, initial) != 0) {
        free(s);
        return NULL;
    }
    return s;
}

int gw_sem_take(gw_sem_t *s, int32_t timeout_ms)
{
    if (s == NULL) {
        return GW_ERR_PARAM;
    }
    if (timeout_ms == GW_WAIT_FOREVER) {
        while (sem_wait(&s->h) == -1) {
            if (errno != EINTR) {
                return GW_ERR;
            }
        }
        return GW_OK;
    }
    if (timeout_ms == GW_NO_WAIT) {
        return (sem_trywait(&s->h) == 0) ? GW_OK : GW_ERR_TIMEOUT;
    }
    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += (time_t)(timeout_ms / 1000);
        ts.tv_nsec += (long)((timeout_ms % 1000) * 1000000);
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec  += 1;
            ts.tv_nsec -= 1000000000L;
        }
        while (sem_timedwait(&s->h, &ts) == -1) {
            if (errno == ETIMEDOUT) {
                return GW_ERR_TIMEOUT;
            }
            if (errno != EINTR) {
                return GW_ERR;
            }
        }
    }
    return GW_OK;
}

int gw_sem_release(gw_sem_t *s)
{
    int v = 0;

    if (s == NULL) {
        return GW_ERR_PARAM;
    }
    /* 带上限检查, 防止计数型信号量被重复 release 冲爆 */
    (void)sem_getvalue(&s->h, &v);
    if ((uint32_t)v >= s->max_value) {
        return GW_ERR_FULL;
    }
    return (sem_post(&s->h) == 0) ? GW_OK : GW_ERR;
}

uint32_t gw_sem_value(gw_sem_t *s)
{
    int v = 0;
    if (s == NULL) {
        return 0xFFFFFFFFu;
    }
    (void)sem_getvalue(&s->h, &v);
    return (uint32_t)((v < 0) ? 0 : v);
}

/* ================================================================== */
/* 消息队列：定长消息环形队列                                          */
/* ================================================================== */
struct gw_mq {
    char            name[16];
    uint8_t        *buf;
    size_t          msg_size;
    size_t          capacity;
    size_t          head;      /* 写入位置（消息序号） */
    size_t          tail;      /* 读取位置（消息序号） */
    size_t          count;
    pthread_mutex_t lock;
    pthread_cond_t  not_empty;
    pthread_cond_t  not_full;
    uint32_t        drop_count;   /* 发送超时被丢弃的次数（诊断用） */
};

gw_mq_t *gw_mq_create(const char *name, size_t msg_size, size_t max_msgs)
{
    struct gw_mq *q;

    port_ensure_ready();
    if ((msg_size == 0u) || (max_msgs == 0u)) {
        return NULL;
    }
    q = (struct gw_mq *)calloc(1, sizeof(*q));
    if (q == NULL) {
        return NULL;
    }
    q->buf = (uint8_t *)calloc(max_msgs, msg_size);
    if (q->buf == NULL) {
        free(q);
        return NULL;
    }
    snprintf(q->name, sizeof(q->name), "%s", (name != NULL) ? name : "mq");
    q->msg_size = msg_size;
    q->capacity = max_msgs;
    pthread_mutex_init(&q->lock, NULL);
    pthread_cond_init(&q->not_empty, NULL);
    pthread_cond_init(&q->not_full, NULL);
    return q;
}

static void port_mq_deadline(struct timespec *ts, int32_t timeout_ms)
{
    clock_gettime(CLOCK_REALTIME, ts);
    ts->tv_sec  += (time_t)(timeout_ms / 1000);
    ts->tv_nsec += (long)((timeout_ms % 1000) * 1000000);
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec  += 1;
        ts->tv_nsec -= 1000000000L;
    }
}

int gw_mq_send(gw_mq_t *q, const void *msg, int32_t timeout_ms)
{
    int rc = GW_OK;

    if ((q == NULL) || (msg == NULL)) {
        return GW_ERR_PARAM;
    }
    pthread_mutex_lock(&q->lock);
    while (q->count >= q->capacity) {
        if (timeout_ms == GW_NO_WAIT) {
            q->drop_count++;
            pthread_mutex_unlock(&q->lock);
            return GW_ERR_FULL;
        }
        if (timeout_ms == GW_WAIT_FOREVER) {
            pthread_cond_wait(&q->not_full, &q->lock);
        } else {
            struct timespec ts;
            port_mq_deadline(&ts, timeout_ms);
            if (pthread_cond_timedwait(&q->not_full, &q->lock, &ts) == ETIMEDOUT) {
                q->drop_count++;
                pthread_mutex_unlock(&q->lock);
                return GW_ERR_FULL;
            }
        }
    }
    memcpy(q->buf + (q->head * q->msg_size), msg, q->msg_size);
    q->head = (q->head + 1u) % q->capacity;
    q->count++;
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->lock);
    return rc;
}

int gw_mq_recv(gw_mq_t *q, void *msg, int32_t timeout_ms)
{
    if ((q == NULL) || (msg == NULL)) {
        return GW_ERR_PARAM;
    }
    pthread_mutex_lock(&q->lock);
    while (q->count == 0u) {
        if (timeout_ms == GW_NO_WAIT) {
            pthread_mutex_unlock(&q->lock);
            return GW_ERR_EMPTY;
        }
        if (timeout_ms == GW_WAIT_FOREVER) {
            pthread_cond_wait(&q->not_empty, &q->lock);
        } else {
            struct timespec ts;
            port_mq_deadline(&ts, timeout_ms);
            if (pthread_cond_timedwait(&q->not_empty, &q->lock, &ts) == ETIMEDOUT) {
                pthread_mutex_unlock(&q->lock);
                return GW_ERR_TIMEOUT;
            }
        }
    }
    memcpy(msg, q->buf + (q->tail * q->msg_size), q->msg_size);
    q->tail = (q->tail + 1u) % q->capacity;
    q->count--;
    pthread_cond_signal(&q->not_full);
    pthread_mutex_unlock(&q->lock);
    return GW_OK;
}

size_t gw_mq_count(gw_mq_t *q)
{
    size_t n;
    if (q == NULL) {
        return 0u;
    }
    pthread_mutex_lock(&q->lock);
    n = q->count;
    pthread_mutex_unlock(&q->lock);
    return n;
}

size_t gw_mq_capacity(gw_mq_t *q)
{
    return (q != NULL) ? q->capacity : 0u;
}

/* ================================================================== */
/* 事件标志组                                                          */
/* ================================================================== */
struct gw_event {
    char            name[16];
    uint32_t        flags;
    pthread_mutex_t lock;
    pthread_cond_t  cv;
};

gw_event_t *gw_event_create(const char *name)
{
    struct gw_event *e;

    port_ensure_ready();
    e = (struct gw_event *)calloc(1, sizeof(*e));
    if (e == NULL) {
        return NULL;
    }
    snprintf(e->name, sizeof(e->name), "%s", (name != NULL) ? name : "evt");
    pthread_mutex_init(&e->lock, NULL);
    pthread_cond_init(&e->cv, NULL);
    return e;
}

uint32_t gw_event_send(gw_event_t *e, uint32_t flags)
{
    uint32_t now;
    if (e == NULL) {
        return 0u;
    }
    pthread_mutex_lock(&e->lock);
    e->flags |= flags;
    now = e->flags;
    pthread_cond_broadcast(&e->cv);
    pthread_mutex_unlock(&e->lock);
    return now;
}

uint32_t gw_event_recv(gw_event_t *e, uint32_t interest, bool wait_all,
                       int32_t timeout_ms)
{
    uint32_t got = 0u;

    if ((e == NULL) || (interest == 0u)) {
        return 0u;
    }

    pthread_mutex_lock(&e->lock);
    for (;;) {
        bool satisfied = wait_all ? ((e->flags & interest) == interest)
                                  : ((e->flags & interest) != 0u);
        if (satisfied) {
            got = e->flags & interest;
            break;
        }
        if (timeout_ms == GW_NO_WAIT) {
            got = 0u;
            break;
        }
        if (timeout_ms == GW_WAIT_FOREVER) {
            pthread_cond_wait(&e->cv, &e->lock);
        } else {
            struct timespec ts;
            port_mq_deadline(&ts, timeout_ms);
            if (pthread_cond_timedwait(&e->cv, &e->lock, &ts) == ETIMEDOUT) {
                got = e->flags & interest;
                break;
            }
        }
    }
    pthread_mutex_unlock(&e->lock);
    return got;
}

uint32_t gw_event_clear(gw_event_t *e, uint32_t flags)
{
    uint32_t left;
    if (e == NULL) {
        return 0u;
    }
    pthread_mutex_lock(&e->lock);
    e->flags &= ~flags;
    left = e->flags;
    pthread_mutex_unlock(&e->lock);
    return left;
}

/* ================================================================== */
/* 看门狗                                                              */
/* ================================================================== */
static struct {
    uint32_t           timeout_ms;
    uint32_t           mask;
    uint32_t           fed;
    uint32_t           last_full_ms;
    bool               inited;
    uint32_t           miss_count;
    gw_wdt_reset_hook_t hook;
} s_wdt;

int gw_wdt_init(uint32_t timeout_ms)
{
    memset(&s_wdt, 0, sizeof(s_wdt));
    s_wdt.timeout_ms   = timeout_ms;
    s_wdt.mask         = 0u;
    s_wdt.last_full_ms = gw_port_tick_ms();
    s_wdt.inited       = true;
    return GW_OK;
}

void gw_wdt_set_mask(uint32_t thread_mask)
{
    s_wdt.mask = thread_mask;
}

void gw_wdt_set_reset_hook(gw_wdt_reset_hook_t hook)
{
    s_wdt.hook = hook;
}

void gw_wdt_feed(uint32_t thread_bit)
{
    s_wdt.fed |= thread_bit;
    if (s_wdt.mask == 0u) {
        /* 尚未登记被监控线程：视为"未启用监控", 只刷新时间戳 */
        s_wdt.last_full_ms = gw_port_tick_ms();
        return;
    }
    if ((s_wdt.fed & s_wdt.mask) == s_wdt.mask) {
        s_wdt.fed          = 0u;
        s_wdt.last_full_ms = gw_port_tick_ms();
    }
}

bool gw_wdt_check(void)
{
    uint32_t now;

    if (!s_wdt.inited) {
        return true;
    }
    now = gw_port_tick_ms();
    if ((now - s_wdt.last_full_ms) <= s_wdt.timeout_ms) {
        return true;
    }

    /* 超时：找出没喂狗的线程位（诊断信息比"看门狗复位"本身更有价值）*/
    s_wdt.miss_count++;
    if (s_wdt.hook != NULL) {
        s_wdt.hook(s_wdt.mask & ~s_wdt.fed, "watchdog timeout: thread(s) did not feed");
    }
    s_wdt.fed          = 0u;
    s_wdt.last_full_ms = now;
    return false;
}
