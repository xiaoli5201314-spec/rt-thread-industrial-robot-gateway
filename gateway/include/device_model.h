/*
 * device_model.h - 数据模型层：设备注册、心跳管理、参数在线映射、缓存队列
 *
 * 这一层是"协议无关"的：无论设备挂在 Modbus RTU、Modbus TCP 还是
 * CANopen 上, 上层服务看到的都是统一的 gw_device_t 与工程值。
 * 好处是新增一种总线时, 只需要加一个协议适配器并注册设备,
 * 服务层（上报/存储/诊断）一行都不用改。
 *
 * 提供三张内建设备档案（数字焊机 / 机器人 / PLC 远程 IO）,
 * 对应 README 中的"Modbus 寄存器映射表"。
 */
#ifndef GW_DEVICE_MODEL_H
#define GW_DEVICE_MODEL_H

#include "gw_types.h"
#include "gateway_config.h"
#include "port_rtos.h"
#include "bus_health.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 设备档案（profile） ---------------- */
#define GW_PROFILE_UNKNOWN       0u
#define GW_PROFILE_WELDER        1u
#define GW_PROFILE_ROBOT         2u
#define GW_PROFILE_PLC           3u
#define GW_PROFILE_CANOPEN_IO    4u

/* 参数数据类型 */
#define GW_PARAM_U8              5u
#define GW_PARAM_U16             0u
#define GW_PARAM_S16             1u
#define GW_PARAM_U32             2u   /* 双寄存器, 高字在前 */
#define GW_PARAM_S32             3u
#define GW_PARAM_BITFIELD        4u

typedef struct {
    uint16_t    reg;          /* 寄存器地址 / 对象字典索引             */
    uint8_t     subindex;     /* CANopen 子索引（Modbus 时为 0）       */
    uint16_t    param_id;     /* 网关内部参数 ID（上报时使用）          */
    uint8_t     type;
    int32_t     scale_num;    /* 工程值 = 原始值 * num / den           */
    int32_t     scale_den;
    const char *name;
    const char *unit;
} gw_param_field_t;

typedef struct {
    uint8_t              profile;
    const char          *name;
    gw_protocol_t        default_proto;
    const gw_param_field_t *fields;
    uint32_t             field_count;
    uint16_t             poll_base;   /* 周期轮询起始寄存器            */
    uint16_t             poll_count;  /* 周期轮询寄存器数量            */
    uint16_t             input_base;  /* 输入寄存器起始（3xxxx）        */
    uint16_t             input_count;
} gw_dev_profile_t;

const gw_dev_profile_t *gw_devm_profile(uint8_t profile);
const gw_param_field_t *gw_devm_field_by_param(uint8_t profile, uint16_t param_id);
const gw_param_field_t *gw_devm_field_by_reg(uint8_t profile, uint16_t reg);

/* ---------------- 设备 ---------------- */
#define GW_DEV_MAX_REGS  32u

typedef struct {
    bool           used;
    uint8_t        id;              /* 网关内部设备号（1..N）           */
    uint8_t        addr;            /* 总线地址 / CANopen 节点号        */
    gw_protocol_t  proto;
    uint8_t        profile;
    char           name[GW_DEVICE_NAME_MAX];
    gw_dev_state_t state;
    uint32_t       last_seen_ms;
    uint32_t       heartbeat_timeout_ms;
    uint32_t       poll_period_ms;
    uint16_t       reg_base;
    uint16_t       reg_count;
    uint16_t       snapshot[GW_DEV_MAX_REGS];   /* 最近一次读回的寄存器   */
    bool           snapshot_valid;

    /* 通信统计 */
    uint32_t poll_ok;
    uint32_t poll_fail;
    uint32_t timeouts;
    uint32_t crc_errors;
    uint32_t retries;
    uint32_t offline_count;
    uint32_t online_count;
    uint32_t state_change_ms;
} gw_device_t;

/* ---------------- 参数写缓存队列 ---------------- */
typedef struct {
    bool     used;
    uint8_t  dev_id;
    uint16_t reg;
    uint16_t value;
    uint8_t  retry;
    uint32_t queued_ms;
} gw_param_cache_item_t;

typedef struct {
    gw_device_t          devices[GW_MAX_DEVICES];
    uint32_t             device_count;
    uint8_t              next_id;

    gw_param_cache_item_t cache[GW_PARAM_CACHE_DEPTH];
    uint32_t             cache_count;
    uint32_t             cache_drop;      /* 队列满丢弃次数              */
    uint32_t             cache_flushed;   /* 成功回放次数                */

    /* 统计 */
    uint32_t total_polls;
    uint32_t total_ok;
    uint32_t total_fail;
} gw_devm_t;

int  gw_devm_init(gw_devm_t *m);
int  gw_devm_register(gw_devm_t *m, uint8_t addr, gw_protocol_t proto,
                      uint8_t profile, const char *name, uint32_t now_ms);
gw_device_t *gw_devm_find(gw_devm_t *m, uint8_t dev_id);
gw_device_t *gw_devm_find_by_addr(gw_devm_t *m, uint8_t addr, gw_protocol_t proto);

/* 状态机：心跳 / 轮询结果驱动的状态迁移 */
void gw_devm_set_state(gw_devm_t *m, gw_device_t *d, gw_dev_state_t st, uint32_t now_ms);
int  gw_devm_heartbeat(gw_devm_t *m, uint8_t dev_id, uint32_t now_ms);
int  gw_devm_check_timeouts(gw_devm_t *m, uint32_t now_ms);
void gw_devm_report_poll_ok(gw_devm_t *m, gw_device_t *d, uint32_t now_ms);
void gw_devm_report_poll_fail(gw_devm_t *m, gw_device_t *d, gw_err_class_t cls,
                              uint32_t now_ms);
/* 保存一次读回的数据（协议适配层调用） */
int  gw_devm_apply_read(gw_devm_t *m, gw_device_t *d, uint16_t reg_base,
                        const uint16_t *values, uint16_t count, uint32_t now_ms);

/* 参数在线映射：把寄存器原始值换算成工程值 */
int  gw_devm_read_param(gw_devm_t *m, uint8_t dev_id, uint16_t param_id,
                        int32_t *eng_value);
/* 参数写入：设备在线直接下发, 离线进入缓存队列（掉线不丢参数） */
int  gw_devm_write_param(gw_devm_t *m, uint8_t dev_id, uint16_t param_id,
                         int32_t eng_value, uint32_t now_ms);
int  gw_devm_cache_push(gw_devm_t *m, uint8_t dev_id, uint16_t reg, uint16_t value,
                        uint32_t now_ms);
int  gw_devm_cache_pop(gw_devm_t *m, gw_param_cache_item_t *out);
uint32_t gw_devm_cache_count(gw_devm_t *m);
int  gw_devm_cache_release(gw_devm_t *m, uint8_t dev_id);

const char *gw_devm_state_str(gw_dev_state_t s);

#ifdef __cplusplus
}
#endif

#endif /* GW_DEVICE_MODEL_H */
