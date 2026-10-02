/*
 * device_model.c - 数据模型层实现
 *
 * 三张内建设备档案的内容直接对应现场三台典型设备, 也是 README 里
 * "Modbus 寄存器映射表"的数据来源。所有换算用整数比例完成
 * （工程值 = 原始值 * num / den）, 避免在实时链路上引入浮点运算
 * 与由此带来的非确定性执行时间。
 *
 * 32 位量（U32/S32）在 Modbus 上占两个连续寄存器, 高字在前（big-endian
 * word order）——这是与主流焊机/机器人控制器互通时的约定, 也是现场最
 * 常见的坑之一, 因此在快照里显式定义。
 */
#include "device_model.h"
#include "gw_log.h"

#include <stdio.h>
#include <string.h>

/* ================================================================== */
/* 档案 1：数字焊机（保持寄存器 0x0000 起、输入寄存器 0x0000 起）      */
/* ================================================================== */
static const gw_param_field_t s_welder_fields[] = {
    { 0x0000u, 0u, 0x1001u, GW_PARAM_U16, 1,  10, "welding_current_set", "0.1A"    },
    { 0x0001u, 0u, 0x1002u, GW_PARAM_U16, 1,  10, "welding_voltage_set", "0.1V"    },
    { 0x0002u, 0u, 0x1003u, GW_PARAM_U16, 1, 100, "wire_feed_speed",     "0.01m/min"},
    { 0x0003u, 0u, 0x1004u, GW_PARAM_U16, 1,  10, "welding_current_act", "0.1A"    },
    { 0x0004u, 0u, 0x1005u, GW_PARAM_U16, 1,  10, "welding_voltage_act", "0.1V"    },
    { 0x0005u, 0u, 0x1006u, GW_PARAM_U16, 1,   1, "weld_state",          "-"       },
    { 0x0006u, 0u, 0x1007u, GW_PARAM_U16, 1,   1, "fault_code",          "-"       },
    { 0x0007u, 0u, 0x1008u, GW_PARAM_U32, 1,   1, "weld_time_total",     "s"       },
    { 0x0009u, 0u, 0x1009u, GW_PARAM_U16, 1,  10, "gas_flow",            "0.1L/min"},
    { 0x000Au, 0u, 0x100Au, GW_PARAM_U16, 1,   1, "welder_enable",       "-"       },
    { 0x000Bu, 0u, 0x100Bu, GW_PARAM_U16, 1,   1, "process_no",          "-"       },
    { 0x000Cu, 0u, 0x100Cu, GW_PARAM_S16, 1,  10, "arc_force",           "0.1"     },
};

static const gw_param_field_t s_welder_input_fields[] = {
    { 0x0000u, 0u, 0x1101u, GW_PARAM_U16, 1,  10, "dc_bus_voltage",     "0.1V"  },
    { 0x0001u, 0u, 0x1102u, GW_PARAM_U16, 1,   1, "igbt_temp",          "C"     },
    { 0x0002u, 0u, 0x1103u, GW_PARAM_U16, 1, 100, "wire_motor_current", "0.01A" },
};

/* ================================================================== */
/* 档案 2：六轴工业机器人（保持寄存器 0x0000 起, 独立地址空间）        */
/*   注意：Modbus 每台从站有**自己的**寄存器地址空间, 因此焊机与机器人 */
/*   都可以从 0x0000 开始编号, 由从站地址区分。这一点在混线现场经常    */
/*   被误解为"地址冲突", 这里刻意按标准做法保留。                      */
/* ================================================================== */
static const gw_param_field_t s_robot_fields[] = {
    { 0x0000u, 0u, 0x2001u, GW_PARAM_U16, 1,   1, "run_mode",      "-"       },
    { 0x0001u, 0u, 0x2002u, GW_PARAM_U16, 1,   1, "program_no",    "-"       },
    { 0x0002u, 0u, 0x2003u, GW_PARAM_S16, 1, 100, "joint1_angle",  "0.01deg" },
    { 0x0003u, 0u, 0x2004u, GW_PARAM_S16, 1, 100, "joint2_angle",  "0.01deg" },
    { 0x0004u, 0u, 0x2005u, GW_PARAM_S16, 1, 100, "joint3_angle",  "0.01deg" },
    { 0x0005u, 0u, 0x2006u, GW_PARAM_S16, 1, 100, "joint4_angle",  "0.01deg" },
    { 0x0006u, 0u, 0x2007u, GW_PARAM_S16, 1, 100, "joint5_angle",  "0.01deg" },
    { 0x0007u, 0u, 0x2008u, GW_PARAM_S16, 1, 100, "joint6_angle",  "0.01deg" },
    { 0x0008u, 0u, 0x2009u, GW_PARAM_S32, 1, 100, "tcp_x",         "0.01mm"  },
    { 0x000Au, 0u, 0x200Au, GW_PARAM_S32, 1, 100, "tcp_y",         "0.01mm"  },
    { 0x000Cu, 0u, 0x200Bu, GW_PARAM_S32, 1, 100, "tcp_z",         "0.01mm"  },
    { 0x000Eu, 0u, 0x200Cu, GW_PARAM_U16, 1,   1, "alarm_code",    "-"       },
    { 0x000Fu, 0u, 0x200Du, GW_PARAM_U32, 1,   1, "cycle_count",   "-"       },
    { 0x0011u, 0u, 0x200Eu, GW_PARAM_U16, 1,   1, "servo_enable",  "-"       },
};

/* ================================================================== */
/* 档案 3：PLC 远程 IO / 线体控制器（保持寄存器 0x0000 起）            */
/* ================================================================== */
static const gw_param_field_t s_plc_fields[] = {
    { 0x0000u, 0u, 0x3001u, GW_PARAM_U16,      1,  10, "line_cycle_time", "0.1s"   },
    { 0x0001u, 0u, 0x3002u, GW_PARAM_U32,      1,   1, "workpiece_count", "-"      },
    { 0x0003u, 0u, 0x3003u, GW_PARAM_BITFIELD, 1,  1, "station_status",  "-"       },
    { 0x0004u, 0u, 0x3004u, GW_PARAM_U16,      1,   1, "estop_state",     "-"      },
    { 0x0005u, 0u, 0x3005u, GW_PARAM_U16,      1,   1, "safety_door",     "-"      },
    { 0x0006u, 0u, 0x3006u, GW_PARAM_U16,      1,  10, "air_pressure",    "0.1bar" },
    { 0x0007u, 0u, 0x3007u, GW_PARAM_S16,      1,  10, "ambient_temp",    "0.1C"   },
};

/* ================================================================== */
/* 档案 4：CANopen 远程 IO 节点（按对象字典索引寻址）                  */
/* ================================================================== */
static const gw_param_field_t s_canopen_io_fields[] = {
    { 0x6000u, 1u, 0x4001u, GW_PARAM_U8,  1, 1, "do_ch1",  "-" },
    { 0x6000u, 2u, 0x4002u, GW_PARAM_U8,  1, 1, "do_ch2",  "-" },
    { 0x6000u, 3u, 0x4003u, GW_PARAM_U8,  1, 1, "do_ch3",  "-" },
    { 0x6000u, 4u, 0x4004u, GW_PARAM_U8,  1, 1, "do_ch4",  "-" },
    { 0x6401u, 1u, 0x4011u, GW_PARAM_U16, 1, 1, "di_ch1_8", "-" },
    { 0x6401u, 2u, 0x4012u, GW_PARAM_U16, 1, 1, "di_ch9_16","-" },
};

static const gw_dev_profile_t s_profiles[] = {
    { GW_PROFILE_WELDER, "digital-welder", GW_PROTO_MODBUS_RTU,
      s_welder_fields, GW_ARRAY_SIZE(s_welder_fields), 0x0000u, 13u, 0x0000u, 3u },
    { GW_PROFILE_ROBOT, "six-axis-robot", GW_PROTO_MODBUS_RTU,
      s_robot_fields, GW_ARRAY_SIZE(s_robot_fields), 0x0000u, 18u, 0u, 0u },
    { GW_PROFILE_PLC, "plc-remote-io", GW_PROTO_MODBUS_TCP,
      s_plc_fields, GW_ARRAY_SIZE(s_plc_fields), 0x0000u, 8u, 0u, 0u },
    { GW_PROFILE_CANOPEN_IO, "canopen-io-node", GW_PROTO_CANOPEN,
      s_canopen_io_fields, GW_ARRAY_SIZE(s_canopen_io_fields), 0u, 0u, 0u, 0u },
};

/* 焊机输入寄存器档案单独挂在一张从表里, 通过 param_id 0x11xx 检索 */
static const struct {
    const gw_param_field_t *fields;
    uint32_t                count;
} s_extra_tables[] = {
    { s_welder_input_fields, GW_ARRAY_SIZE(s_welder_input_fields) },
};

const gw_dev_profile_t *gw_devm_profile(uint8_t profile)
{
    uint32_t i;
    for (i = 0u; i < GW_ARRAY_SIZE(s_profiles); i++) {
        if (s_profiles[i].profile == profile) {
            return &s_profiles[i];
        }
    }
    return NULL;
}

const gw_param_field_t *gw_devm_field_by_param(uint8_t profile, uint16_t param_id)
{
    const gw_dev_profile_t *p = gw_devm_profile(profile);
    uint32_t i;
    uint32_t t;

    if (p != NULL) {
        for (i = 0u; i < p->field_count; i++) {
            if (p->fields[i].param_id == param_id) {
                return &p->fields[i];
            }
        }
    }
    /* 输入寄存器表对所有档案共享（3xxxx 区） */
    for (t = 0u; t < GW_ARRAY_SIZE(s_extra_tables); t++) {
        for (i = 0u; i < s_extra_tables[t].count; i++) {
            if (s_extra_tables[t].fields[i].param_id == param_id) {
                return &s_extra_tables[t].fields[i];
            }
        }
    }
    return NULL;
}

const gw_param_field_t *gw_devm_field_by_reg(uint8_t profile, uint16_t reg)
{
    const gw_dev_profile_t *p = gw_devm_profile(profile);
    uint32_t i;

    if (p == NULL) {
        return NULL;
    }
    for (i = 0u; i < p->field_count; i++) {
        if (p->fields[i].reg == reg) {
            return &p->fields[i];
        }
    }
    return NULL;
}

const char *gw_devm_state_str(gw_dev_state_t s)
{
    return gw_dev_state_str(s);
}

/* ================================================================== */
/* 设备管理                                                            */
/* ================================================================== */
int gw_devm_init(gw_devm_t *m)
{
    if (m == NULL) {
        return GW_ERR_PARAM;
    }
    memset(m, 0, sizeof(*m));
    m->next_id = 1u;
    return GW_OK;
}

int gw_devm_register(gw_devm_t *m, uint8_t addr, gw_protocol_t proto,
                     uint8_t profile, const char *name, uint32_t now_ms)
{
    uint32_t i;
    const gw_dev_profile_t *p;

    if (m == NULL) {
        return GW_ERR_PARAM;
    }
    if (gw_devm_find_by_addr(m, addr, proto) != NULL) {
        return GW_ERR_STATE;              /* 同一总线地址重复注册 */
    }

    for (i = 0u; i < GW_MAX_DEVICES; i++) {
        if (!m->devices[i].used) {
            gw_device_t *d = &m->devices[i];
            memset(d, 0, sizeof(*d));
            d->used                 = true;
            d->id                   = m->next_id++;
            d->addr                 = addr;
            d->proto                = proto;
            d->profile              = profile;
            d->state                = GW_DEV_UNKNOWN;
            d->last_seen_ms         = now_ms;
            d->heartbeat_timeout_ms = GW_HEARTBEAT_TIMEOUT_MS;
            d->poll_period_ms       = GW_POLL_PERIOD_MS;
            d->state_change_ms      = now_ms;
            p = gw_devm_profile(profile);
            if (p != NULL) {
                d->reg_base  = p->poll_base;
                d->reg_count = (p->poll_count > GW_DEV_MAX_REGS)
                             ? GW_DEV_MAX_REGS : p->poll_count;
                if (d->reg_base == 0u) {
                    d->reg_base = p->fields[0].reg;
                }
            }
            snprintf(d->name, sizeof(d->name), "%s",
                     (name != NULL) ? name : ((p != NULL) ? p->name : "device"));
            m->device_count++;
            GW_LOGI("DEVM", "registered dev#%u addr=%u proto=%s profile=%s",
                    (unsigned)d->id, (unsigned)addr, gw_protocol_str(proto),
                    (p != NULL) ? p->name : "?");
            return (int)d->id;
        }
    }
    return GW_ERR_FULL;
}

gw_device_t *gw_devm_find(gw_devm_t *m, uint8_t dev_id)
{
    uint32_t i;

    if (m == NULL) {
        return NULL;
    }
    for (i = 0u; i < GW_MAX_DEVICES; i++) {
        if (m->devices[i].used && (m->devices[i].id == dev_id)) {
            return &m->devices[i];
        }
    }
    return NULL;
}

gw_device_t *gw_devm_find_by_addr(gw_devm_t *m, uint8_t addr, gw_protocol_t proto)
{
    uint32_t i;

    if (m == NULL) {
        return NULL;
    }
    for (i = 0u; i < GW_MAX_DEVICES; i++) {
        if (m->devices[i].used && (m->devices[i].addr == addr) &&
            (m->devices[i].proto == proto)) {
            return &m->devices[i];
        }
    }
    return NULL;
}

void gw_devm_set_state(gw_devm_t *m, gw_device_t *d, gw_dev_state_t st, uint32_t now_ms)
{
    GW_UNUSED(m);
    if ((d == NULL) || (d->state == st)) {
        return;
    }
    GW_LOGI("DEVM", "dev#%u %s -> %s", (unsigned)d->id,
            gw_dev_state_str(d->state), gw_dev_state_str(st));
    d->state           = st;
    d->state_change_ms = now_ms;
}

int gw_devm_heartbeat(gw_devm_t *m, uint8_t dev_id, uint32_t now_ms)
{
    gw_device_t *d = gw_devm_find(m, dev_id);

    if (d == NULL) {
        return GW_ERR_NOT_FOUND;
    }
    d->last_seen_ms = now_ms;
    if (d->state != GW_DEV_ONLINE) {
        d->online_count++;
        gw_devm_set_state(m, d, GW_DEV_ONLINE, now_ms);
    }
    return GW_OK;
}

int gw_devm_check_timeouts(gw_devm_t *m, uint32_t now_ms)
{
    uint32_t i;
    int      newly_offline = 0;

    if (m == NULL) {
        return GW_ERR_PARAM;
    }
    for (i = 0u; i < GW_MAX_DEVICES; i++) {
        gw_device_t *d = &m->devices[i];
        if (!d->used) {
            continue;
        }
        if ((now_ms - d->last_seen_ms) > d->heartbeat_timeout_ms) {
            if ((d->state == GW_DEV_ONLINE) || (d->state == GW_DEV_DEGRADED)) {
                d->offline_count++;
                gw_devm_set_state(m, d, GW_DEV_OFFLINE, now_ms);
                newly_offline++;
            }
        }
    }
    return newly_offline;
}

void gw_devm_report_poll_ok(gw_devm_t *m, gw_device_t *d, uint32_t now_ms)
{
    if ((m == NULL) || (d == NULL)) {
        return;
    }
    m->total_polls++;
    m->total_ok++;
    d->poll_ok++;
    d->last_seen_ms = now_ms;
    if ((d->state != GW_DEV_ONLINE) || (d->state == GW_DEV_DEGRADED)) {
        gw_devm_set_state(m, d, GW_DEV_ONLINE, now_ms);
    }
}

void gw_devm_report_poll_fail(gw_devm_t *m, gw_device_t *d, gw_err_class_t cls,
                              uint32_t now_ms)
{
    if ((m == NULL) || (d == NULL)) {
        return;
    }
    m->total_polls++;
    m->total_fail++;
    d->poll_fail++;

    switch (cls) {
    case GW_ERRCLASS_TIMEOUT:  d->timeouts++;   break;
    case GW_ERRCLASS_CRC:      d->crc_errors++; break;
    default:                                    break;
    }

    if ((d->state == GW_DEV_ONLINE) && (d->poll_fail >= 2u)) {
        gw_devm_set_state(m, d, GW_DEV_DEGRADED, now_ms);
    }
}

int gw_devm_apply_read(gw_devm_t *m, gw_device_t *d, uint16_t reg_base,
                       const uint16_t *values, uint16_t count, uint32_t now_ms)
{
    uint16_t i;

    GW_UNUSED(m);
    if ((d == NULL) || (values == NULL) || (count == 0u)) {
        return GW_ERR_PARAM;
    }
    if ((uint32_t)count > GW_DEV_MAX_REGS) {
        return GW_ERR_OVERFLOW;
    }

    for (i = 0u; i < count; i++) {
        d->snapshot[i] = values[i];
    }
    d->reg_base       = reg_base;
    d->reg_count      = count;
    d->snapshot_valid = true;
    d->last_seen_ms   = now_ms;
    return GW_OK;
}

/* 从快照里取一个 16/32 位原始值 */
static int devm_raw_value(gw_device_t *d, const gw_param_field_t *f,
                          int32_t *raw, bool *is_signed)
{
    uint32_t offset;

    if ((d == NULL) || (f == NULL) || (raw == NULL) || (is_signed == NULL)) {
        return GW_ERR_PARAM;
    }
    if (!d->snapshot_valid || (f->reg < d->reg_base)) {
        return GW_ERR_NOT_READY;
    }
    offset = (uint32_t)(f->reg - d->reg_base);
    if (offset >= d->reg_count) {
        return GW_ERR_NOT_FOUND;
    }

    *is_signed = false;
    switch (f->type) {
    case GW_PARAM_U8:
    case GW_PARAM_U16:
    case GW_PARAM_BITFIELD:
        *raw = (int32_t)d->snapshot[offset];
        return GW_OK;
    case GW_PARAM_S16:
        *raw = (int32_t)(int16_t)d->snapshot[offset];
        *is_signed = true;
        return GW_OK;
    case GW_PARAM_U32:
    case GW_PARAM_S32: {
        uint32_t v;
        if ((offset + 1u) >= d->reg_count) {
            return GW_ERR_NOT_FOUND;
        }
        /* 高字在前（与主流控制器一致） */
        v = ((uint32_t)d->snapshot[offset] << 16) |
            (uint32_t)d->snapshot[offset + 1u];
        if (f->type == GW_PARAM_S32) {
            *raw = (int32_t)v;
            *is_signed = true;
        } else {
            *raw = (int32_t)v;
        }
        return GW_OK;
    }
    default:
        return GW_ERR_UNSUPPORTED;
    }
}

int gw_devm_read_param(gw_devm_t *m, uint8_t dev_id, uint16_t param_id,
                       int32_t *eng_value)
{
    gw_device_t            *d = gw_devm_find(m, dev_id);
    const gw_param_field_t *f;
    int32_t                 raw = 0;
    bool                    is_signed = false;
    int                     rc;

    if ((d == NULL) || (eng_value == NULL)) {
        return GW_ERR_NOT_FOUND;
    }
    f = gw_devm_field_by_param(d->profile, param_id);
    if (f == NULL) {
        return GW_ERR_NOT_FOUND;
    }

    rc = devm_raw_value(d, f, &raw, &is_signed);
    if (rc != GW_OK) {
        return rc;
    }
    GW_UNUSED(is_signed);

    if (f->scale_den != 0) {
        *eng_value = (raw * f->scale_num) / f->scale_den;
    } else {
        *eng_value = raw;
    }
    return GW_OK;
}

int gw_devm_cache_push(gw_devm_t *m, uint8_t dev_id, uint16_t reg, uint16_t value,
                       uint32_t now_ms)
{
    uint32_t i;

    if (m == NULL) {
        return GW_ERR_PARAM;
    }
    /* 同一寄存器只保留最后一次设定值（后写覆盖前写, 避免回放一堆中间态） */
    for (i = 0u; i < GW_PARAM_CACHE_DEPTH; i++) {
        if (m->cache[i].used && (m->cache[i].dev_id == dev_id) &&
            (m->cache[i].reg == reg)) {
            m->cache[i].value     = value;
            m->cache[i].queued_ms = now_ms;
            m->cache[i].retry     = 0u;
            return GW_OK;
        }
    }
    for (i = 0u; i < GW_PARAM_CACHE_DEPTH; i++) {
        if (!m->cache[i].used) {
            m->cache[i].used      = true;
            m->cache[i].dev_id    = dev_id;
            m->cache[i].reg       = reg;
            m->cache[i].value     = value;
            m->cache[i].retry     = 0u;
            m->cache[i].queued_ms = now_ms;
            m->cache_count++;
            return GW_OK;
        }
    }
    m->cache_drop++;
    return GW_ERR_FULL;
}

int gw_devm_cache_pop(gw_devm_t *m, gw_param_cache_item_t *out)
{
    uint32_t i;

    if ((m == NULL) || (out == NULL)) {
        return GW_ERR_PARAM;
    }
    for (i = 0u; i < GW_PARAM_CACHE_DEPTH; i++) {
        if (m->cache[i].used) {
            *out = m->cache[i];
            m->cache[i].used = false;
            if (m->cache_count > 0u) {
                m->cache_count--;
            }
            m->cache_flushed++;
            return GW_OK;
        }
    }
    return GW_ERR_EMPTY;
}

uint32_t gw_devm_cache_count(gw_devm_t *m)
{
    return (m != NULL) ? m->cache_count : 0u;
}

int gw_devm_cache_release(gw_devm_t *m, uint8_t dev_id)
{
    uint32_t i;
    int      n = 0;

    if (m == NULL) {
        return GW_ERR_PARAM;
    }
    for (i = 0u; i < GW_PARAM_CACHE_DEPTH; i++) {
        if (m->cache[i].used && (m->cache[i].dev_id == dev_id)) {
            m->cache[i].used = false;
            if (m->cache_count > 0u) {
                m->cache_count--;
            }
            n++;
        }
    }
    return n;
}

int gw_devm_write_param(gw_devm_t *m, uint8_t dev_id, uint16_t param_id,
                        int32_t eng_value, uint32_t now_ms)
{
    gw_device_t            *d = gw_devm_find(m, dev_id);
    const gw_param_field_t *f;
    int32_t                 raw;

    if (d == NULL) {
        return GW_ERR_NOT_FOUND;
    }
    f = gw_devm_field_by_param(d->profile, param_id);
    if (f == NULL) {
        return GW_ERR_NOT_FOUND;
    }
    if (f->scale_den == 0) {
        raw = eng_value;
    } else {
        raw = (eng_value * f->scale_den) / f->scale_num;
    }

    if (raw < 0) {
        raw = (int32_t)(uint16_t)(int16_t)raw;
    }
    if (raw > 0xFFFF) {
        return GW_ERR_PARAM;
    }

    if (d->state != GW_DEV_ONLINE) {
        /* 设备不在线：进入缓存队列, 等心跳恢复后由存储线程回放 */
        return gw_devm_cache_push(m, dev_id, f->reg, (uint16_t)raw, now_ms);
    }
    return GW_OK;   /* 在线：由协议适配层立即下发, 不排队 */
}
