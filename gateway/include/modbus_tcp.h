/*
 * modbus_tcp.h - Modbus TCP：MBAP 头 + 报文组装 + 主站
 *
 * MBAP 头（7 字节）：
 *   +-------------+-------------+----------+-----------+-----------+
 *   | Transaction |  Protocol   |  Length  |  Unit ID  |   PDU     |
 *   |  2 字节     |  2 字节(=0) |  2 字节  |  1 字节   |  变长     |
 *   +-------------+-------------+----------+-----------+-----------+
 *
 * 与 RTU 最大的区别：没有 CRC, 靠 Length 字段定界。
 * 因此 TCP 侧的"粘包/分包"是长度字段问题, 而不是静默间隔问题 ——
 * 这在架构上正好和 RTU 的帧同步状态机形成互补, 也解释了为什么
 * 网关需要同时维护两套定界逻辑。
 */
#ifndef GW_MODBUS_TCP_H
#define GW_MODBUS_TCP_H

#include "gw_types.h"
#include "gateway_config.h"
#include "modbus.h"
#include "port_hw.h"
#include "modbus_rtu.h"    /* 复用 gw_mb_slave_t 寄存器表 */

#ifdef __cplusplus
extern "C" {
#endif

#define GW_MBAP_LEN        7u
#define GW_MB_TCP_DEFAULT_PORT 502

typedef struct {
    uint16_t transaction_id;
    uint16_t protocol_id;
    uint16_t length;      /* Unit ID + PDU 的字节数                   */
    uint8_t  unit_id;
} gw_mbap_t;

int gw_mb_tcp_build(uint16_t tid, uint8_t unit, const uint8_t *pdu,
                    uint32_t pdu_len, uint8_t *out, uint32_t cap,
                    uint32_t *out_len);
int gw_mb_tcp_parse(const uint8_t *adu, uint32_t len, gw_mbap_t *hdr,
                    const uint8_t **pdu, uint32_t *pdu_len);

/* ---------------- 报文组装（解决 TCP 拆包/粘包） ---------------- */
typedef void (*gw_mb_tcp_frame_cb)(void *ctx, const uint8_t *adu, uint32_t len);

typedef struct {
    uint8_t  buf[GW_MAX_ADU_TCP];
    uint32_t len;
    uint32_t frames_ok;
    uint32_t bytes_in;
    uint32_t oversize;
    uint32_t bad_protocol;
    uint32_t resync;
} gw_mb_tcp_asm_t;

void gw_mb_tcp_asm_init(gw_mb_tcp_asm_t *a);
void gw_mb_tcp_asm_feed(gw_mb_tcp_asm_t *a, const uint8_t *data, uint32_t len,
                        gw_mb_tcp_frame_cb cb, void *ctx);

/* ---------------- 主站 ---------------- */
typedef struct {
    gw_link_t       *link;
    uint32_t         timeout_ms;
    uint32_t         retries;
    uint16_t         next_tid;
    gw_mb_tcp_asm_t  asm_;
    uint8_t          frame[GW_MAX_ADU_TCP];
    uint32_t         frame_len;
    bool             got_frame;
    /* 可选帧来源：与 RTU 主站同一套签名, 由 DMA+环形缓冲+MBAP 组装器供帧 */
    int            (*frame_source)(void *ctx, uint8_t *frame, uint32_t cap,
                                   uint32_t *len, uint32_t timeout_ms);
    void            *frame_source_ctx;

    uint32_t         tx_frames;
    uint32_t         rx_frames;
    uint32_t         ok_frames;
    uint32_t         timeouts;
    uint32_t         retry_count;
    uint32_t         exceptions;
    uint32_t         tid_mismatch;
    uint32_t         last_attempts;   /* 最近一次事务实际尝试次数 */
    int              last_error;
    uint8_t          last_exception;
} gw_mb_tcp_master_t;

int gw_mb_tcp_master_init(gw_mb_tcp_master_t *m, gw_link_t *link,
                          uint32_t timeout_ms, uint32_t retries);
void gw_mb_tcp_master_set_frame_source(gw_mb_tcp_master_t *m,
                                       int (*fn)(void *ctx, uint8_t *frame,
                                                 uint32_t cap, uint32_t *len,
                                                 uint32_t timeout_ms),
                                       void *ctx);
/* 通用事务：返回原始响应 ADU（MBAP + PDU）, 便于上层自行解析/转发 */
int gw_mb_tcp_master_txn(gw_mb_tcp_master_t *m, uint8_t unit, const uint8_t *pdu,
                         uint32_t pdu_len, uint8_t *resp_adu, uint32_t cap,
                         uint32_t *resp_len);
int gw_mb_tcp_master_read(gw_mb_tcp_master_t *m, uint8_t unit, uint8_t fc,
                          uint16_t addr, uint16_t qty, uint16_t *out);
int gw_mb_tcp_master_write_single(gw_mb_tcp_master_t *m, uint8_t unit,
                                  uint16_t addr, uint16_t value);
int gw_mb_tcp_master_write_multi(gw_mb_tcp_master_t *m, uint8_t unit,
                                 uint16_t addr, const uint16_t *values,
                                 uint16_t qty);

/* ---------------- 从站（进程内模拟, 复用寄存器表） ---------------- */
int gw_mb_tcp_slave_process(gw_mb_slave_t *s, const uint8_t *req, uint32_t req_len,
                            uint8_t *resp, uint32_t cap, uint32_t *resp_len);

#ifdef __cplusplus
}
#endif

#endif /* GW_MODBUS_TCP_H */
