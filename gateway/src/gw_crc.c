/*
 * gw_crc.c - CRC 实现
 *
 * 逐位实现与查表实现并存的原因：
 *   - 逐位实现代码量 < 100 字节, 不需要 512 字节常量表, 适合 boot 阶段
 *     与 Flash 紧张的场景；
 *   - 查表实现时间恒定（无数据相关的分支）, 在 115200bps 连续收帧的
 *     热路径上可预测性更好。
 * 单元测试中两者对 4096 组随机数据做交叉校验, 保证不会出现"表抄错"
 * 这类只在现场才暴露的问题。
 */
#include "gw_crc.h"

/* ------------------------------------------------------------------ */
/* 逐位实现（反射多项式 0xA001）                                       */
/* ------------------------------------------------------------------ */
uint16_t gw_crc16_modbus_update(uint16_t crc, const uint8_t *data, size_t len)
{
    size_t i;
    uint8_t bit;

    if (data == NULL) {
        return crc;
    }

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (bit = 0; bit < 8u; bit++) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

uint16_t gw_crc16_modbus(const uint8_t *data, size_t len)
{
    return gw_crc16_modbus_update(0xFFFFu, data, len);
}

/* ------------------------------------------------------------------ */
/* 查表实现                                                            */
/* ------------------------------------------------------------------ */
static uint16_t s_crc_table[256];
static bool     s_table_ready = false;

static uint16_t crc16_bitwise_u8(uint16_t crc, uint8_t byte)
{
    uint8_t bit;
    crc ^= (uint16_t)byte;
    for (bit = 0; bit < 8u; bit++) {
        crc = ((crc & 0x0001u) != 0u) ? (uint16_t)((crc >> 1) ^ 0xA001u)
                                      : (uint16_t)(crc >> 1);
    }
    return crc;
}

static void crc_table_init(void)
{
    uint16_t i;
    for (i = 0; i < 256u; i++) {
        s_crc_table[i] = crc16_bitwise_u8(0x0000u, (uint8_t)i);
    }
    s_table_ready = true;
}

uint16_t gw_crc16_modbus_fast(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    size_t   i;

    if (data == NULL) {
        return crc;
    }
    if (!s_table_ready) {
        crc_table_init();
    }

    for (i = 0; i < len; i++) {
        uint8_t idx = (uint8_t)((crc ^ (uint16_t)data[i]) & 0x00FFu);
        crc = (uint16_t)((crc >> 8) ^ s_crc_table[idx]);
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/* 帧尾 CRC 操作（Modbus 规程：低字节先发）                            */
/* ------------------------------------------------------------------ */
bool gw_crc16_modbus_check_frame(const uint8_t *frame, size_t len)
{
    uint16_t calc;
    uint16_t on_wire;

    if ((frame == NULL) || (len < 4u)) {
        return false;
    }

    calc    = gw_crc16_modbus_fast(frame, len - 2u);
    on_wire = (uint16_t)((uint16_t)frame[len - 2u] |
                         ((uint16_t)frame[len - 1u] << 8));

    return (calc == on_wire);
}

void gw_crc16_modbus_append(uint8_t *frame, size_t len_without_crc)
{
    uint16_t crc;

    if (frame == NULL) {
        return;
    }
    crc = gw_crc16_modbus_fast(frame, len_without_crc);
    frame[len_without_crc]      = (uint8_t)(crc & 0x00FFu);
    frame[len_without_crc + 1u] = (uint8_t)((crc >> 8) & 0x00FFu);
}

/* ------------------------------------------------------------------ */
/* CRC-8 (poly 0x07, init 0x00, 无反射无最终异或)                      */
/* 用于参数存储页头的完整性校验（比 CRC16 更短的头部开销）             */
/* ------------------------------------------------------------------ */
uint8_t gw_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00u;
    size_t  i;
    uint8_t bit;

    if (data == NULL) {
        return crc;
    }

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (bit = 0; bit < 8u; bit++) {
            crc = ((crc & 0x80u) != 0u) ? (uint8_t)((uint8_t)(crc << 1) ^ 0x07u)
                                        : (uint8_t)(crc << 1);
        }
    }
    return crc;
}
