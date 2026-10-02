/*
 * bus_health.h - 总线错误分级处理与设备隔离/恢复
 *
 * 简历"设计总线错误分级处理（非法帧 / CRC 错 / 设备掉线 / 总线短路）"
 * 的落地。核心思想：
 *   1) 错误必须分类。四类错误的现场含义完全不同：
 *        - 非法帧   -> 软件/配置问题（从站不支持该功能码, 地址写错）
 *        - CRC 错   -> 电气干扰/接地/终端电阻问题
 *        - 掉线超时 -> 设备断电、线缆断、从站死机
 *        - 总线短路 -> 硬件故障, 必须立刻停止发送, 否则可能损坏收发器
 *   2) 单点故障不能拖垮整条链路。某台设备连续出错就把它"隔离"，
 *      其余设备继续轮询；隔离不是永久拉黑, 冷却期后降级为低频探测,
 *      收到连续 N 个好帧即自动恢复。
 *   3) 所有判决都要留痕：事件日志环形缓冲 + 统计计数, 便于现场复盘。
 */
#ifndef GW_BUS_HEALTH_H
#define GW_BUS_HEALTH_H

#include "gw_types.h"
#include "gateway_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 错误分类 ---------------- */
typedef enum {
    GW_ERRCLASS_NONE = 0,
    GW_ERRCLASS_ILLEGAL_FRAME,     /* 非法帧：长度/功能码/字节数不合法  */
    GW_ERRCLASS_CRC,               /* CRC 校验失败                      */
    GW_ERRCLASS_TIMEOUT,           /* 无响应 / 设备掉线                 */
    GW_ERRCLASS_BUS_SHORT,         /* 总线短路、过流、Bus-Off           */
    GW_ERRCLASS_EXCEPTION,         /* 从站返回协议异常响应（有答复）    */
    GW_ERRCLASS_STACK_OVERFLOW,    /* 线程栈溢出（软件可靠性事件）      */
    GW_ERRCLASS_COUNT
} gw_err_class_t;

const char *gw_err_class_str(gw_err_class_t c);
/* 该错误是否需要计入"设备级隔离"判决（异常响应属于正常协议行为, 不计） */
bool gw_err_class_is_fatal(gw_err_class_t c);

/* ---------------- 判决动作 ---------------- */
typedef enum {
    GW_BUS_ACTION_NONE = 0,
    GW_BUS_ACTION_WARN,            /* 仅计数告警                        */
    GW_BUS_ACTION_DEGRADE,         /* 总线降级（降低轮询频率）          */
    GW_BUS_ACTION_ISOLATE,         /* 隔离该设备                        */
    GW_BUS_ACTION_RECOVER,         /* 设备重新纳入轮询                  */
    GW_BUS_ACTION_BUS_FAULT,       /* 总线级故障：停止发送              */
    GW_BUS_ACTION_BUS_RECOVER      /* 总线故障解除                      */
} gw_bus_action_t;

const char *gw_bus_action_str(gw_bus_action_t a);

/* ---------------- 事件日志 ---------------- */
typedef struct {
    uint32_t        timestamp_ms;
    uint8_t         dev_id;
    gw_err_class_t  err_class;
    gw_bus_action_t action;
    uint32_t        counter;        /* 触发时的连续错误计数              */
} gw_bus_event_t;

/* ---------------- 每设备健康记录 ---------------- */
typedef struct {
    bool     used;
    uint8_t  dev_id;
    uint32_t consec_crc;            /* 连续 CRC 错                        */
    uint32_t consec_illegal;        /* 连续非法帧                         */
    uint32_t consec_timeout;        /* 连续超时                           */
    uint32_t consec_good;           /* 连续正常帧                         */
    uint32_t total_errors;
    uint32_t total_good;
    bool     isolated;
    uint32_t isolate_ts_ms;
    uint32_t isolate_count;         /* 累计被隔离次数                     */
    uint32_t recover_count;         /* 累计恢复次数                       */
    uint32_t probe_count;           /* 隔离期间的探测次数                 */
} gw_bus_dev_health_t;

typedef struct {
    gw_err_class_t cls;
    uint32_t       total;
    uint32_t       per_class[GW_ERRCLASS_COUNT];
} gw_err_stats_t;

typedef struct {
    gw_bus_state_t      state;
    gw_bus_dev_health_t devs[GW_MAX_DEVICES];
    uint32_t            dev_count;

    /* 滑动窗口错误率（用于降级判决） */
    uint32_t window_start_ms;
    uint32_t window_errors;
    uint32_t window_total;
    uint32_t ratio_x100;            /* 最近一次计算的错误率 * 100         */

    /* 总线级故障 */
    bool     bus_fault;
    uint32_t bus_fault_count;
    uint32_t bus_fault_ts_ms;
    uint32_t bus_recover_good;      /* 故障解除所需连续好帧计数           */

    /* 统计 */
    gw_err_stats_t stats;
    uint32_t isolate_total;
    uint32_t recover_total;
    uint32_t degrade_total;
    uint32_t hw_fault_mask;

    /* 事件日志（环形） */
    gw_bus_event_t events[GW_EVENT_LOG_DEPTH];
    uint32_t       event_head;
    uint32_t       event_total;
} gw_bus_health_t;

int  gw_bus_health_init(gw_bus_health_t *bh);
int  gw_bus_health_register_dev(gw_bus_health_t *bh, uint8_t dev_id);
gw_bus_dev_health_t *gw_bus_health_dev(gw_bus_health_t *bh, uint8_t dev_id);

/* 上报一次错误；返回系统采取的判决动作 */
gw_bus_action_t gw_bus_health_report(gw_bus_health_t *bh, uint8_t dev_id,
                                     gw_err_class_t cls, uint32_t now_ms);
/* 上报一个正常帧；返回 GW_BUS_ACTION_RECOVER 表示设备已重新纳入轮询 */
gw_bus_action_t gw_bus_health_report_good(gw_bus_health_t *bh, uint8_t dev_id,
                                          uint32_t now_ms);
/* 硬件故障信号（来自 port_hw 的故障位图） */
gw_bus_action_t gw_bus_health_hw_fault(gw_bus_health_t *bh, uint32_t fault_mask,
                                       uint32_t now_ms);

/* 是否允许对该设备发起轮询；隔离期内返回 false, 冷却期满返回 true（探测） */
bool gw_bus_health_should_poll(gw_bus_health_t *bh, uint8_t dev_id, uint32_t now_ms);
bool gw_bus_health_is_isolated(gw_bus_health_t *bh, uint8_t dev_id);

gw_bus_state_t gw_bus_health_state(gw_bus_health_t *bh);
void           gw_bus_health_get_stats(gw_bus_health_t *bh, gw_err_stats_t *out);
/* 读取事件日志第 i 条（0 = 最近一条） */
bool gw_bus_health_get_event(gw_bus_health_t *bh, uint32_t back_index,
                             gw_bus_event_t *out);
/* 周期性调用：窗口错误率计算 + 状态跃迁 */
gw_bus_state_t gw_bus_health_tick(gw_bus_health_t *bh, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* GW_BUS_HEALTH_H */
