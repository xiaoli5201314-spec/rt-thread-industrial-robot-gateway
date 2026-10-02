/*
 * modbus_rtu.h - Modbus RTU：ADU 编解码 + 主站事务 + 从站模拟
 *
 * ADU 结构（最大 256 字节）：
 *   +--------+-----------+---------------+----------+
 *   | Slave  | Function  |    Data ...   |  CRC16   |
 *   | 1 字节 |  1 字节   |   0..252 字节 | 2 字节   |
 *   +--------+-----------+---------------+----------+
 *   CRC 低字节先发（Modbus 规程 Serial Line IP 章节）。
 *
 * 主站事务流程（这是"超时重试"的落地）：
 *   1) 组帧 -> 发送
 *   2) 等 200ms（GW_MB_RESP_TIMEOUT_MS）：期间把收到的字节喂给帧同步
 *   3) 帧同步给出 CRC 正确的帧 -> 校验从站地址与功能码 -> 成功
 *   4) 超时 / CRC 错 -> 重试, 最多 GW_MB_RETRY_MAX 次
 *   5) 从站返回异常响应（FC|0x80）-> 不重试, 直接上报异常码
 *      （异常是"从站明确答复", 重试没有意义, 反而拖慢总线）
 */
#ifndef GW_MODBUS_RTU_H
#define GW_MODBUS_RTU_H

#include "gw_types.h"
#include "gateway_config.h"
#include "modbus.h"
#include "frame_sync.h"
#include "port_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 从站寄存器表容量（与设备模型中的映射表对应） */
#define GW_MB_HOLDING_REGS   512u
#define GW_MB_INPUT_REGS     128u

/* 链路接收分片大小：一次 recv 的上限, 128 字节在 115200bps 下约 11ms,
 * 既不会让单次中断处理过长, 也不会因为分片太碎而频繁进入帧同步。 */
#define GW_MB_RX_CHUNK       128u

/* ================================================================== */
/* 期望长度推断：供帧同步状态机做早期判定                              */
/* ================================================================== */
uint32_t gw_mb_rtu_expected_len(const uint8_t *buf, uint32_t len);

/* ================================================================== */
/* ADU 编解码                                                          */
/* ================================================================== */
int gw_mb_rtu_build_read(uint8_t slave, uint8_t fc, uint16_t addr,
                         uint16_t qty, uint8_t *out, uint32_t cap,
                         uint32_t *out_len);
int gw_mb_rtu_build_write_single(uint8_t slave, uint16_t addr, uint16_t value,
                                 uint8_t *out, uint32_t cap, uint32_t *out_len);
int gw_mb_rtu_build_write_multi(uint8_t slave, uint16_t addr,
                                const uint16_t *values, uint16_t qty,
                                uint8_t *out, uint32_t cap, uint32_t *out_len);

/* 解析请求 ADU（含 CRC 与地址检查） */
int gw_mb_rtu_parse_request(const uint8_t *adu, uint32_t len,
                            gw_mb_request_t *out);
/* 解析响应 ADU（含 CRC 与地址检查） */
int gw_mb_rtu_parse_response(const uint8_t *adu, uint32_t len,
                             gw_mb_response_t *out);

/* ================================================================== */
/* 主站                                                                */
/* ================================================================== */
typedef struct {
    gw_frame_sync_t fs;
    uint8_t         buf[GW_FRAME_SYNC_BUF];
    uint8_t         frame[GW_MAX_ADU_RTU];
    uint32_t        frame_len;
    bool            got_frame;
    /* 接收侧质量统计, 最终汇入总线健康度 */
    uint32_t        crc_errors;
    uint32_t        short_frames;
    uint32_t        resync_count;
} gw_mb_rtu_rx_t;

typedef struct {
    gw_link_t        *link;
    uint32_t          timeout_ms;
    uint32_t          retries;
    gw_mb_rtu_rx_t    rx;
    /*
     * 可选的"帧来源"：目标板上由 DMA+环形缓冲+帧同步状态机提供帧，
     * ISR 只负责把字节搬进环形缓冲并释放信号量, 协议线程在这里取帧。
     * 为 NULL 时主站直接从 link 收字节并自己做帧同步（PC 单测路径）。
     * 两条路径共用同一套超时/重试/异常处理逻辑, 避免维护两份。
     */
    int             (*frame_source)(void *ctx, uint8_t *frame, uint32_t cap,
                                    uint32_t *len, uint32_t timeout_ms);
    void             *frame_source_ctx;
    /* 统计（verify_protocol.py 与诊断线程都读这里） */
    uint32_t          tx_frames;
    uint32_t          rx_frames;
    uint32_t          ok_frames;
    uint32_t          timeouts;
    uint32_t          retry_count;
    uint32_t          crc_errors;
    uint32_t          short_frames;
    uint32_t          exceptions;
    uint32_t          bad_slave;
    uint32_t          source_timeouts;   /* 帧来源超时次数              */
    uint32_t          last_attempts;     /* 最近一次事务实际尝试了几次   */
    int               last_error;      /* gw_mb_error_t                */
    uint8_t           last_exception;
} gw_mb_rtu_master_t;

int gw_mb_rtu_master_init(gw_mb_rtu_master_t *m, gw_link_t *link,
                          uint32_t timeout_ms, uint32_t retries);
/* 挂接"帧来源"（DMA+环形缓冲路径）。fn 返回 GW_OK 表示取到一帧 */
void gw_mb_rtu_master_set_frame_source(gw_mb_rtu_master_t *m,
                                       int (*fn)(void *ctx, uint8_t *frame,
                                                 uint32_t cap, uint32_t *len,
                                                 uint32_t timeout_ms),
                                       void *ctx);

/* 通用事务：req 为完整 ADU, 内部自动重试 */
int gw_mb_rtu_master_txn(gw_mb_rtu_master_t *m, const uint8_t *req,
                         uint32_t req_len, uint8_t *resp, uint32_t cap,
                         uint32_t *resp_len);

/* 语义化封装 */
int gw_mb_rtu_master_read(gw_mb_rtu_master_t *m, uint8_t slave, uint8_t fc,
                          uint16_t addr, uint16_t qty, uint16_t *out,
                          uint16_t *out_qty);
int gw_mb_rtu_master_write_single(gw_mb_rtu_master_t *m, uint8_t slave,
                                  uint16_t addr, uint16_t value);
int gw_mb_rtu_master_write_multi(gw_mb_rtu_master_t *m, uint8_t slave,
                                 uint16_t addr, const uint16_t *values,
                                 uint16_t qty);

/* ================================================================== */
/* 从站（进程内模拟从站, 单元测试与 gw_host_sim --inproc 使用）        */
/* ================================================================== */
typedef struct {
    uint8_t  addr;                                  /* 本从站地址        */
    uint16_t holding[GW_MB_HOLDING_REGS];           /* 保持寄存器 4xxxx  */
    uint16_t input[GW_MB_INPUT_REGS];               /* 输入寄存器 3xxxx  */
    uint16_t holding_count;                         /* 有效寄存器数量    */
    uint16_t input_count;
    /* 统计 */
    uint32_t requests_handled;
    uint32_t exceptions_sent;
    uint32_t crc_rejected;
    /* 现场可维护的"从站能力位"：不支持的功能码回 01 异常 */
    uint32_t supported_fc_mask;
} gw_mb_slave_t;

int  gw_mb_slave_init(gw_mb_slave_t *s, uint8_t addr, uint16_t holding_count,
                      uint16_t input_count);
/* 处理一个请求 ADU, 生成响应 ADU；返回 GW_OK 或负错误码 */
int  gw_mb_slave_process(gw_mb_slave_t *s, const uint8_t *req, uint32_t req_len,
                         uint8_t *resp, uint32_t cap, uint32_t *resp_len);

#ifdef __cplusplus
}
#endif

#endif /* GW_MODBUS_RTU_H */
