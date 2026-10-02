/*
 * param_store.h - 掉电参数保存与上电自恢复
 *
 * 策略：NOR Flash 上开两个等长槽位（2KB x 2）, 每次保存写到"非当前"槽,
 * 写完回读校验, 校验通过才把该槽置为当前。这样任何时刻至少有一个槽
 * 是完整的：
 *   - 写一半掉电  -> 新槽 CRC 坏, 上电时自动回退到旧槽（recovery_count++）
 *   - 擦除后掉电  -> 同上
 *   - 连续正常保存 -> 两槽轮换, 单槽擦写次数减半, 延长 Flash 寿命
 *
 * 槽头结构（16 字节, 4 字节对齐）：
 *   magic(4) | seq(4) | payload_len(2) | hdr_crc16(2) | payload_crc16(2) | rsv(2)
 */
#ifndef GW_PARAM_STORE_H
#define GW_PARAM_STORE_H

#include "gw_types.h"
#include "port_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GW_PARAM_MAGIC        0x47575031u   /* "GWP1" */
#define GW_PARAM_SLOT_SIZE    2048u
#define GW_PARAM_SLOT_COUNT   2u
#define GW_PARAM_MAX_PAYLOAD  (GW_PARAM_SLOT_SIZE - 32u)

typedef struct {
    uint32_t active_slot;
    uint32_t seq;
    uint32_t save_count;
    uint32_t load_count;
    uint32_t corrupt_count;      /* 校验失败的槽次数（含掉电打断）      */
    uint32_t recovery_count;     /* 因主槽损坏而回退到备份槽的次数      */
    uint32_t factory_reset_count;
    uint32_t last_len;
    bool     loaded;
} gw_param_store_t;

/* 参数区内容（网关的可掉电保持参数） */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t device_count;
    uint32_t poll_period_ms;
    uint32_t heartbeat_timeout_ms;
    uint32_t bus_retry_max;
    uint32_t isolate_cooldown_ms;
    uint32_t welder_current_set;
    uint32_t welder_voltage_set;
    uint32_t wire_feed_speed;
    uint32_t robot_program_no;
    uint32_t report_interval_ms;
    uint32_t crc_seed;
} gw_param_blob_t;

int  gw_param_store_init(gw_param_store_t *ps);
int  gw_param_save(gw_param_store_t *ps, const void *payload, uint16_t len);
int  gw_param_load(gw_param_store_t *ps, void *payload, uint16_t cap,
                   uint16_t *out_len);
int  gw_param_factory_reset(gw_param_store_t *ps);
void gw_param_store_stats(gw_param_store_t *ps, gw_param_store_t *out);

/* 默认参数（首次上电 / 恢复出厂） */
void gw_param_defaults(gw_param_blob_t *blob);

#ifdef __cplusplus
}
#endif

#endif /* GW_PARAM_STORE_H */
