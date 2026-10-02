/*
 * port_rtos.h - RTOS 抽象层（唯一的平台相关接口）
 *
 * 设计目标：
 *   同一份 gateway 源码既能编译到 RT-Thread 目标板, 也能在 PC 上以
 *   pthread 仿真运行, 从而让协议栈、状态机、内存池等逻辑可以在 CI 中
 *   被真实执行与断言, 而不是"写完就算"。
 *
 * 优先级约定（与 RT-Thread 一致，必须牢记）：
 *   *** 数值越小优先级越高 ***
 *   0   = 最高优先级（RT-Thread 保留给中断相关线程的区间为 0..2 左右）
 *   31  = 最低优先级
 * 因此 GW_PRIO_HIGH(8) 比 GW_PRIO_LOW(24) 更"高"。
 */
#ifndef GW_PORT_RTOS_H
#define GW_PORT_RTOS_H

#include "gw_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 无限等待 / 不等待（所有带 timeout_ms 的接口通用） */
#define GW_WAIT_FOREVER   ((int32_t)-1)
#define GW_NO_WAIT        ((int32_t)0)

/* ------------------------------------------------------------------ */
/* 不透明句柄                                                          */
/* ------------------------------------------------------------------ */
typedef struct gw_thread gw_thread_t;
typedef struct gw_mutex  gw_mutex_t;
typedef struct gw_sem    gw_sem_t;
typedef struct gw_mq     gw_mq_t;
typedef struct gw_event  gw_event_t;

typedef void (*gw_thread_entry_t)(void *param);

/* ------------------------------------------------------------------ */
/* 端口初始化与时间基准                                                */
/* ------------------------------------------------------------------ */
void     gw_port_init(void);
uint32_t gw_port_tick_ms(void);          /* 单调递增毫秒计数            */
uint32_t gw_port_now_us(void);           /* 单调递增微秒计数            */
void     gw_port_delay_ms(uint32_t ms);  /* 让出 CPU 的睡眠             */
void     gw_port_delay_us(uint32_t us);
void     gw_port_console_write(const char *s, size_t len); /* 日志出口   */
const char *gw_port_name(void);          /* "host-pthread" / "rt-thread" */

/* 端口能力查询：PC 端口在拥有 CAP_SYS_NICE 且 GW_RT_SCHED=1 时可切到
 * SCHED_FIFO, 此时"优先级继承"是操作系统真实行为；目标板恒为 true。 */
bool        gw_port_rt_sched_active(void);
const char *gw_port_sched_mode(void);

/* ------------------------------------------------------------------ */
/* 线程                                                                */
/* ------------------------------------------------------------------ */
/*
 * 创建并立即启动一个线程。
 *   stack_bytes : 线程栈大小（字节）, 端口层负责按对齐分配并做栈涂色,
 *                 以便 gw_thread_stack_used() 给出真实高水位。
 *   priority    : 逻辑优先级, 0 最高。数值语义与 RT-Thread 相同。
 * 失败返回 NULL。
 */
gw_thread_t *gw_thread_create(const char *name, gw_thread_entry_t entry,
                              void *param, uint32_t stack_bytes, uint8_t priority);

/* 等待线程退出；GW_WAIT_FOREVER 表示一直等 */
int  gw_thread_join(gw_thread_t *t, int32_t timeout_ms);
void gw_thread_yield(void);

const char *gw_thread_self_name(void);
uint8_t     gw_thread_self_priority(void);

/* 栈高水位（已使用字节数, 含红区）；端口不支持时返回 -1 */
int32_t gw_thread_stack_used(gw_thread_t *t);
int32_t gw_thread_stack_size(gw_thread_t *t);

/* 栈溢出检测：端口启动时会在线程栈底放置涂色红区, 此处校验其完整性 */
bool gw_thread_stack_overflow(gw_thread_t *t);

/* 溢出钩子：由诊断线程注册, 触发后进入总线健康度统计与看门狗上报 */
typedef void (*gw_stack_overflow_hook_t)(const char *thread_name, int32_t stack_size);
void gw_thread_set_overflow_hook(gw_stack_overflow_hook_t hook);

/* ------------------------------------------------------------------ */
/* 互斥锁（递归 + 可选优先级继承）                                     */
/*                                                                     */
/* RT-Thread: rt_mutex_create(name, RT_IPC_FLAG_PRIO)                  */
/*            即"按优先级等待 + 持有者优先级继承", 可避免优先级反转。  */
/* PC 端口:   pthread_mutexattr_setprotocol(PTHREAD_PRIO_INHERIT)      */
/*            叠加本层逻辑优先级提升记录, 使继承过程可被断言与打印。   */
/* ------------------------------------------------------------------ */
gw_mutex_t *gw_mutex_create(const char *name, bool priority_inherit);
int  gw_mutex_take(gw_mutex_t *m, int32_t timeout_ms);
int  gw_mutex_release(gw_mutex_t *m);

/* 优先级继承过程的可观测统计（这是"证明继承生效"的证据来源） */
typedef struct {
    uint32_t take_count;               /* 加锁总次数                        */
    uint32_t contend_count;            /* 发生了等待（竞争）的次数          */
    uint32_t inherit_count;            /* 实际发生优先级提升的次数          */
    uint32_t restore_count;            /* 提升后恢复原优先级的次数          */
    uint32_t max_hold_us;              /* 最长持锁时间（微秒）              */
    uint8_t  last_owner_prio_base;     /* 最近一次: 持有者原始优先级        */
    uint8_t  last_owner_prio_boosted;  /* 最近一次: 提升后的优先级          */
    uint8_t  last_waiter_prio;         /* 最近一次: 触发提升的等待者优先级   */
} gw_mutex_pi_stats_t;

int gw_mutex_pi_stats(gw_mutex_t *m, gw_mutex_pi_stats_t *out);

/* ------------------------------------------------------------------ */
/* 信号量                                                              */
/* ------------------------------------------------------------------ */
gw_sem_t *gw_sem_create(const char *name, uint32_t initial, uint32_t max_value);
int       gw_sem_take(gw_sem_t *s, int32_t timeout_ms);
int       gw_sem_release(gw_sem_t *s);
uint32_t  gw_sem_value(gw_sem_t *s);   /* 不支持时返回 0xFFFFFFFF */

/* ------------------------------------------------------------------ */
/* 消息队列：定长消息、按值拷贝、FIFO                                  */
/* ------------------------------------------------------------------ */
gw_mq_t *gw_mq_create(const char *name, size_t msg_size, size_t max_msgs);
int      gw_mq_send(gw_mq_t *q, const void *msg, int32_t timeout_ms);
int      gw_mq_recv(gw_mq_t *q, void *msg, int32_t timeout_ms);
size_t   gw_mq_count(gw_mq_t *q);      /* 待处理消息数, -1 表示不支持   */
size_t   gw_mq_capacity(gw_mq_t *q);

/* ------------------------------------------------------------------ */
/* 事件标志组（32 位；bit0..bit3 预留给采集/上报/诊断协同）            */
/* ------------------------------------------------------------------ */
gw_event_t *gw_event_create(const char *name);
uint32_t    gw_event_send(gw_event_t *e, uint32_t flags);
/* wait_all=true 表示"与"等待, false 表示"或"等待; 返回生效的标志位 */
uint32_t    gw_event_recv(gw_event_t *e, uint32_t interest, bool wait_all,
                          int32_t timeout_ms);
uint32_t    gw_event_clear(gw_event_t *e, uint32_t flags);

/* ------------------------------------------------------------------ */
/* 看门狗                                                              */
/* ------------------------------------------------------------------ */
typedef void (*gw_wdt_reset_hook_t)(uint32_t missed_mask, const char *reason);

int  gw_wdt_init(uint32_t timeout_ms);
void gw_wdt_feed(uint32_t thread_bit);      /* 每个被监控线程喂自己的位  */
bool gw_wdt_check(void);                    /* 全部喂到返回 true         */
void gw_wdt_set_reset_hook(gw_wdt_reset_hook_t hook);
void gw_wdt_set_mask(uint32_t thread_mask); /* 需要被监控的线程位图      */

#ifdef __cplusplus
}
#endif

#endif /* GW_PORT_RTOS_H */
