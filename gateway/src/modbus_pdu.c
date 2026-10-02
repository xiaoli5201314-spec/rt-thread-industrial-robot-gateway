/*
 * modbus_pdu.c - Modbus PDU 层编解码（RTU 与 TCP 共用）
 *
 * 所有长度与数量边界都在这里做一次性校验, 上层拿到的请求/响应结构
 * 一定是"已经合法"的, 避免每个调用点重复写防御代码。
 */
#include "modbus.h"

#include <string.h>

const char *gw_mb_error_str(gw_mb_error_t e)
{
    switch (e) {
    case GW_MB_ERR_NONE:         return "NONE";
    case GW_MB_ERR_BAD_CRC:      return "BAD_CRC";
    case GW_MB_ERR_BAD_SLAVE:    return "BAD_SLAVE";
    case GW_MB_ERR_BAD_FUNCTION: return "BAD_FUNCTION";
    case GW_MB_ERR_BAD_LENGTH:   return "BAD_LENGTH";
    case GW_MB_ERR_BAD_ADDRESS:  return "BAD_ADDRESS";
    case GW_MB_ERR_BAD_VALUE:    return "BAD_VALUE";
    case GW_MB_ERR_TIMEOUT:      return "TIMEOUT";
    case GW_MB_ERR_LINK:         return "LINK";
    case GW_MB_ERR_EXCEPTION:    return "EXCEPTION";
    default:                     return "UNKNOWN";
    }
}

const char *gw_mb_exception_str(uint8_t code)
{
    switch (code) {
    case GW_MB_EXC_ILLEGAL_FUNCTION: return "ILLEGAL_FUNCTION";
    case GW_MB_EXC_ILLEGAL_ADDRESS:  return "ILLEGAL_DATA_ADDRESS";
    case GW_MB_EXC_ILLEGAL_VALUE:    return "ILLEGAL_DATA_VALUE";
    case GW_MB_EXC_SLAVE_FAILURE:    return "SLAVE_DEVICE_FAILURE";
    case GW_MB_EXC_ACKNOWLEDGE:      return "ACKNOWLEDGE";
    case GW_MB_EXC_SLAVE_BUSY:       return "SLAVE_DEVICE_BUSY";
    case GW_MB_EXC_NEGATIVE_ACK:     return "NEGATIVE_ACKNOWLEDGE";
    default:                         return "UNKNOWN_EXCEPTION";
    }
}

bool gw_mb_is_exception_function(uint8_t fc)
{
    return (fc & GW_MB_FC_EXCEPTION_MASK) != 0u;
}

bool gw_mb_function_supported(uint8_t fc)
{
    switch (fc) {
    case GW_MB_FC_READ_HOLDING:
    case GW_MB_FC_READ_INPUT:
    case GW_MB_FC_WRITE_SINGLE_REG:
    case GW_MB_FC_WRITE_MULTI_REGS:
        return true;
    case GW_MB_FC_READ_COILS:
    case GW_MB_FC_READ_DISCRETE:
    case GW_MB_FC_WRITE_SINGLE_COIL:
    case GW_MB_FC_WRITE_MULTI_COILS:
        /* 线圈类功能码在网关上不做映射, 一律视为"不支持"，
         * 让主站立刻拿到 01 异常而不是等到超时。 */
        return false;
    default:
        return false;
    }
}

/* ------------------------------------------------------------------ */
/* 构造                                                               */
/* ------------------------------------------------------------------ */
int gw_mb_pdu_build_read(uint8_t fc, uint16_t addr, uint16_t qty,
                         uint8_t *out, uint32_t cap, uint32_t *out_len)
{
    if ((out == NULL) || (out_len == NULL) || (cap < 5u)) {
        return GW_ERR_PARAM;
    }
    if ((fc != GW_MB_FC_READ_HOLDING) && (fc != GW_MB_FC_READ_INPUT)) {
        return GW_ERR_PARAM;
    }
    if ((qty == 0u) || (qty > GW_MODBUS_MAX_READ_REGS)) {
        return GW_ERR_PARAM;
    }

    out[0] = fc;
    gw_write_be16(&out[1], addr);
    gw_write_be16(&out[3], qty);
    *out_len = 5u;
    return GW_OK;
}

int gw_mb_pdu_build_write_single(uint16_t addr, uint16_t value,
                                 uint8_t *out, uint32_t cap, uint32_t *out_len)
{
    if ((out == NULL) || (out_len == NULL) || (cap < 5u)) {
        return GW_ERR_PARAM;
    }
    out[0] = GW_MB_FC_WRITE_SINGLE_REG;
    gw_write_be16(&out[1], addr);
    gw_write_be16(&out[3], value);
    *out_len = 5u;
    return GW_OK;
}

int gw_mb_pdu_build_write_multi(uint16_t addr, const uint16_t *values,
                                uint16_t qty, uint8_t *out, uint32_t cap,
                                uint32_t *out_len)
{
    uint32_t need;
    uint16_t i;

    if ((out == NULL) || (out_len == NULL) || (values == NULL)) {
        return GW_ERR_PARAM;
    }
    if ((qty == 0u) || (qty > GW_MODBUS_MAX_WRITE_REGS)) {
        return GW_ERR_PARAM;
    }
    need = 6u + ((uint32_t)qty * 2u);
    if (cap < need) {
        return GW_ERR_PARAM;
    }

    out[0] = GW_MB_FC_WRITE_MULTI_REGS;
    gw_write_be16(&out[1], addr);
    gw_write_be16(&out[3], qty);
    out[5] = (uint8_t)(qty * 2u);
    for (i = 0u; i < qty; i++) {
        gw_write_be16(&out[6u + ((uint32_t)i * 2u)], values[i]);
    }
    *out_len = need;
    return GW_OK;
}

int gw_mb_pdu_build_read_response(uint8_t fc, const uint16_t *values,
                                  uint16_t qty, uint8_t *out, uint32_t cap,
                                  uint32_t *out_len)
{
    uint32_t need;
    uint16_t i;

    if ((out == NULL) || (out_len == NULL) || (values == NULL)) {
        return GW_ERR_PARAM;
    }
    if ((qty == 0u) || (qty > GW_MODBUS_MAX_READ_REGS)) {
        return GW_ERR_PARAM;
    }
    need = 2u + ((uint32_t)qty * 2u);
    if (cap < need) {
        return GW_ERR_PARAM;
    }

    out[0] = fc;
    out[1] = (uint8_t)(qty * 2u);
    for (i = 0u; i < qty; i++) {
        gw_write_be16(&out[2u + ((uint32_t)i * 2u)], values[i]);
    }
    *out_len = need;
    return GW_OK;
}

int gw_mb_pdu_build_write_response(uint8_t fc, uint16_t addr,
                                   uint16_t value_or_qty, uint8_t *out,
                                   uint32_t cap, uint32_t *out_len)
{
    if ((out == NULL) || (out_len == NULL) || (cap < 5u)) {
        return GW_ERR_PARAM;
    }
    out[0] = fc;
    gw_write_be16(&out[1], addr);
    gw_write_be16(&out[3], value_or_qty);
    *out_len = 5u;
    return GW_OK;
}

int gw_mb_pdu_build_exception(uint8_t fc, uint8_t code, uint8_t *out,
                              uint32_t cap, uint32_t *out_len)
{
    if ((out == NULL) || (out_len == NULL) || (cap < 2u)) {
        return GW_ERR_PARAM;
    }
    out[0] = (uint8_t)(fc | GW_MB_FC_EXCEPTION_MASK);
    out[1] = code;
    *out_len = 2u;
    return GW_OK;
}

/* ------------------------------------------------------------------ */
/* 解析：请求（从站侧）                                                */
/* ------------------------------------------------------------------ */
int gw_mb_pdu_parse_request(const uint8_t *pdu, uint32_t len,
                            gw_mb_request_t *out)
{
    uint8_t  fc;
    uint16_t i;

    if ((pdu == NULL) || (out == NULL) || (len < 1u)) {
        return GW_ERR_PARAM;
    }

    memset(out, 0, sizeof(*out));
    fc = pdu[0];
    out->function = fc;

    if (!gw_mb_function_supported(fc)) {
        return GW_ERR_UNSUPPORTED;
    }

    switch (fc) {
    case GW_MB_FC_READ_HOLDING:
    case GW_MB_FC_READ_INPUT:
        if (len != 5u) {
            return GW_ERR_PARAM;
        }
        out->start_addr = gw_read_be16(&pdu[1]);
        out->quantity   = gw_read_be16(&pdu[3]);
        if ((out->quantity == 0u) || (out->quantity > GW_MODBUS_MAX_READ_REGS)) {
            return GW_ERR_PARAM;
        }
        break;

    case GW_MB_FC_WRITE_SINGLE_REG:
        if (len != 5u) {
            return GW_ERR_PARAM;
        }
        out->start_addr = gw_read_be16(&pdu[1]);
        out->quantity   = 1u;
        out->value      = gw_read_be16(&pdu[3]);
        break;

    case GW_MB_FC_WRITE_MULTI_REGS:
        if (len < 6u) {
            return GW_ERR_PARAM;
        }
        out->start_addr = gw_read_be16(&pdu[1]);
        out->quantity   = gw_read_be16(&pdu[3]);
        out->byte_count = pdu[5];
        if ((out->quantity == 0u) || (out->quantity > GW_MODBUS_MAX_WRITE_REGS)) {
            return GW_ERR_PARAM;
        }
        /* 字节数必须与寄存器数量严格一致, 长度也必须精确吻合,
         * 这是防止"粘包/截断帧"被误当成合法写入的关键校验 */
        if ((uint32_t)out->byte_count != ((uint32_t)out->quantity * 2u)) {
            return GW_ERR_PARAM;
        }
        if (len != 6u + (uint32_t)out->byte_count) {
            return GW_ERR_PARAM;
        }
        for (i = 0u; i < out->quantity; i++) {
            out->values[i] = gw_read_be16(&pdu[6u + ((uint32_t)i * 2u)]);
        }
        break;

    default:
        return GW_ERR_UNSUPPORTED;
    }

    return GW_OK;
}

/* ------------------------------------------------------------------ */
/* 解析：响应（主站侧）                                                */
/* ------------------------------------------------------------------ */
int gw_mb_pdu_parse_response(const uint8_t *pdu, uint32_t len,
                             gw_mb_response_t *out)
{
    uint8_t  fc;
    uint16_t i;

    if ((pdu == NULL) || (out == NULL) || (len < 2u)) {
        return GW_ERR_PARAM;
    }

    memset(out, 0, sizeof(*out));
    fc = pdu[0];
    out->function = fc;

    if (gw_mb_is_exception_function(fc)) {
        out->is_exception   = true;
        out->function       = (uint8_t)(fc & ~GW_MB_FC_EXCEPTION_MASK);
        out->exception_code = pdu[1];
        if (len != 2u) {
            return GW_ERR_PARAM;
        }
        return GW_ERR_EXCEPTION;
    }

    switch (fc) {
    case GW_MB_FC_READ_HOLDING:
    case GW_MB_FC_READ_INPUT:
        if (len < 3u) {
            return GW_ERR_PARAM;
        }
        if ((pdu[1] == 0u) || ((pdu[1] & 0x01u) != 0u)) {
            return GW_ERR_PARAM;   /* 字节数必须为偶数且非零 */
        }
        if (len != 2u + (uint32_t)pdu[1]) {
            return GW_ERR_PARAM;
        }
        out->value_count = (uint16_t)(pdu[1] / 2u);
        if (out->value_count > GW_MODBUS_MAX_READ_REGS) {
            return GW_ERR_PARAM;
        }
        for (i = 0u; i < out->value_count; i++) {
            out->values[i] = gw_read_be16(&pdu[2u + ((uint32_t)i * 2u)]);
        }
        break;

    case GW_MB_FC_WRITE_SINGLE_REG:
        if (len != 5u) {
            return GW_ERR_PARAM;
        }
        out->start_addr = gw_read_be16(&pdu[1]);
        out->value      = gw_read_be16(&pdu[3]);
        out->quantity   = 1u;
        out->values[0]  = out->value;
        out->value_count = 1u;
        break;

    case GW_MB_FC_WRITE_MULTI_REGS:
        if (len != 5u) {
            return GW_ERR_PARAM;
        }
        out->start_addr = gw_read_be16(&pdu[1]);
        out->quantity   = gw_read_be16(&pdu[3]);
        break;

    default:
        return GW_ERR_UNSUPPORTED;
    }

    return GW_OK;
}
