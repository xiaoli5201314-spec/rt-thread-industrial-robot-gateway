/*
 * modbus.h - Modbus 协议族共享定义（与传输层无关的 PDU 层）
 *
 * 分层：
 *   PDU  = 功能码 + 数据        （本文件, RTU/TCP 完全一致）
 *   ADU  = 地址 + PDU + CRC     （modbus_rtu.h）
 *   ADU  = MBAP + PDU           （modbus_tcp.h）
 *
 * 只实现工程实际用到的功能码：03/04 读保持与输入寄存器、06 写单个、
 * 0F/10 写多个。其余功能码在从站侧统一回"非法功能码"异常响应,
 * 这比"静默丢弃"更利于现场定位（上层能立刻区分"不支持"与"没应答"）。
 */
#ifndef GW_MODBUS_H
#define GW_MODBUS_H

#include "gw_types.h"
#include "gateway_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 功能码 ---------------- */
#define GW_MB_FC_READ_COILS           0x01u
#define GW_MB_FC_READ_DISCRETE        0x02u
#define GW_MB_FC_READ_HOLDING         0x03u
#define GW_MB_FC_READ_INPUT           0x04u
#define GW_MB_FC_WRITE_SINGLE_COIL    0x05u
#define GW_MB_FC_WRITE_SINGLE_REG     0x06u
#define GW_MB_FC_WRITE_MULTI_COILS    0x0Fu
#define GW_MB_FC_WRITE_MULTI_REGS     0x10u
#define GW_MB_FC_EXCEPTION_MASK       0x80u

/* ---------------- 异常码 ---------------- */
#define GW_MB_EXC_ILLEGAL_FUNCTION    0x01u  /* 不支持该功能码            */
#define GW_MB_EXC_ILLEGAL_ADDRESS     0x02u  /* 地址越界                  */
#define GW_MB_EXC_ILLEGAL_VALUE       0x03u  /* 数量/数值非法             */
#define GW_MB_EXC_SLAVE_FAILURE       0x04u  /* 从站内部故障              */
#define GW_MB_EXC_ACKNOWLEDGE         0x05u  /* 已接受, 需较长时间处理    */
#define GW_MB_EXC_SLAVE_BUSY          0x06u  /* 从站忙                    */
#define GW_MB_EXC_NEGATIVE_ACK        0x07u

/* ---------------- 本地错误码（不占用线上异常码） ---------------- */
typedef enum {
    GW_MB_ERR_NONE         = 0,
    GW_MB_ERR_BAD_CRC      = 1,
    GW_MB_ERR_BAD_SLAVE    = 2,
    GW_MB_ERR_BAD_FUNCTION = 3,
    GW_MB_ERR_BAD_LENGTH   = 4,
    GW_MB_ERR_BAD_ADDRESS  = 5,
    GW_MB_ERR_BAD_VALUE    = 6,
    GW_MB_ERR_TIMEOUT      = 7,
    GW_MB_ERR_LINK         = 8,
    GW_MB_ERR_EXCEPTION    = 9
} gw_mb_error_t;

const char *gw_mb_error_str(gw_mb_error_t e);
const char *gw_mb_exception_str(uint8_t code);

/* ---------------- 解析结果结构 ---------------- */
typedef struct {
    uint8_t  slave;                 /* 从站地址                          */
    uint8_t  function;              /* 功能码                            */
    uint16_t start_addr;            /* 起始寄存器地址                    */
    uint16_t quantity;              /* 寄存器数量                        */
    uint16_t value;                 /* 06: 写入值                        */
    uint8_t  byte_count;            /* 16: 数据字节数                    */
    uint16_t values[GW_MODBUS_MAX_WRITE_REGS]; /* 16: 写入的数据        */
} gw_mb_request_t;

typedef struct {
    uint8_t  slave;
    uint8_t  function;
    bool     is_exception;
    uint8_t  exception_code;
    uint16_t start_addr;
    uint16_t quantity;
    uint16_t value;                 /* 06 回显值                         */
    uint16_t values[GW_MODBUS_MAX_READ_REGS];  /* 03/04 读回的数据      */
    uint16_t value_count;
} gw_mb_response_t;

/* ---------------- PDU 编解码 ---------------- */
/* 构造：写入 out（仅 PDU, 不含地址与 CRC）, 返回 PDU 长度或负错误码 */
int gw_mb_pdu_build_read(uint8_t fc, uint16_t addr, uint16_t qty,
                         uint8_t *out, uint32_t cap, uint32_t *out_len);
int gw_mb_pdu_build_write_single(uint16_t addr, uint16_t value,
                                 uint8_t *out, uint32_t cap, uint32_t *out_len);
int gw_mb_pdu_build_write_multi(uint16_t addr, const uint16_t *values,
                                uint16_t qty, uint8_t *out, uint32_t cap,
                                uint32_t *out_len);

/* 解析请求 PDU（从站侧使用, 不含地址与 CRC） */
int gw_mb_pdu_parse_request(const uint8_t *pdu, uint32_t len,
                            gw_mb_request_t *out);

/* 构造响应 PDU（从站侧使用） */
int gw_mb_pdu_build_read_response(uint8_t fc, const uint16_t *values,
                                  uint16_t qty, uint8_t *out, uint32_t cap,
                                  uint32_t *out_len);
int gw_mb_pdu_build_write_response(uint8_t fc, uint16_t addr,
                                   uint16_t value_or_qty, uint8_t *out,
                                   uint32_t cap, uint32_t *out_len);
int gw_mb_pdu_build_exception(uint8_t fc, uint8_t code, uint8_t *out,
                              uint32_t cap, uint32_t *out_len);

/* 解析响应 PDU（主站侧使用） */
int gw_mb_pdu_parse_response(const uint8_t *pdu, uint32_t len,
                             gw_mb_response_t *out);

/* 功能码合法性（从站侧的"非法功能码"判定依据） */
bool gw_mb_function_supported(uint8_t fc);
bool gw_mb_is_exception_function(uint8_t fc);

#ifdef __cplusplus
}
#endif

#endif /* GW_MODBUS_H */
