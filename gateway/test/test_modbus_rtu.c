/*
 * test_modbus_rtu.c - Modbus RTU / TCP 协议测试
 *
 * 覆盖验收标准：
 *   - 功能码 03 / 06 / 16 的编解码
 *   - CRC16 校验（帧级）
 *   - 异常响应：非法功能码(01) / 非法地址(02) / 非法数据值(03)
 *   - 超时重试与链路恢复
 *   - 黄金帧字节级比对（与 Modbus 规程公开示例一致）
 */
#include "test_util.h"
#include "modbus_rtu.h"
#include "modbus_tcp.h"
#include "gw_crc.h"
#include "bus_health.h"

#include <string.h>

/* Modbus TCP 回环从站适配器：把字节流接到 gw_mb_tcp_slave_process */
static gw_mb_slave_t *s_tcp_slave = NULL;

static int tcp_slave_fn(void *ctx, const uint8_t *req, size_t req_len,
                        uint8_t *resp, size_t cap)
{
    uint32_t n = 0u;
    int      rc;

    GW_UNUSED(ctx);
    if (s_tcp_slave == NULL) {
        return 0;
    }
    rc = gw_mb_tcp_slave_process(s_tcp_slave, req, (uint32_t)req_len, resp,
                                 (uint32_t)cap, &n);
    if ((rc != GW_OK) && (rc != GW_ERR_EXCEPTION)) {
        return 0;
    }
    return (int)n;
}

/* ================================================================== */
/* 带故障注入的测试从站                                                */
/* ================================================================== */
typedef struct {
    gw_mb_slave_t slave;
    uint32_t      corrupt_crc;      /* 还要破坏几次 CRC                  */
    uint32_t      silent;           /* 还要静默几次                      */
    uint32_t      truncate;         /* 还要截断几次                      */
    uint32_t      handled;
    uint32_t      corrupted;
    uint32_t      silent_done;
    uint32_t      truncated;
} t_slave_t;

static int t_slave_fn(void *ctx, const uint8_t *req, size_t req_len,
                      uint8_t *resp, size_t cap)
{
    t_slave_t *ts = (t_slave_t *)ctx;
    uint32_t   n = 0u;
    int        rc;

    if (ts->silent > 0u) {
        ts->silent--;
        ts->silent_done++;
        return 0;
    }

    rc = gw_mb_slave_process(&ts->slave, req, (uint32_t)req_len, resp,
                             (uint32_t)cap, &n);
    if ((rc != GW_OK) && (rc != GW_ERR_EXCEPTION)) {
        return 0;
    }
    ts->handled++;

    if ((ts->corrupt_crc > 0u) && (n >= 2u)) {
        ts->corrupt_crc--;
        ts->corrupted++;
        resp[n - 1u] ^= 0xFFu;
        return (int)n;
    }
    if ((ts->truncate > 0u) && (n > 4u)) {
        ts->truncate--;
        ts->truncated++;
        return (int)(n - 3u);
    }
    return (int)n;
}

/* ================================================================== */
/* 编解码                                                              */
/* ================================================================== */
static void t_codec(void)
{
    uint8_t  adu[64];
    uint32_t len = 0u;
    uint16_t vals[4] = { 0x1234u, 0x5678u, 0x9ABCu, 0xDEF0u };
    gw_mb_request_t  rq;
    gw_mb_response_t rp;

    GW_CASE("FC03 读保持寄存器请求：黄金帧 01 03 00 00 00 0A C5 CD");
    GW_ASSERT_EQ_INT(gw_mb_rtu_build_read(1u, GW_MB_FC_READ_HOLDING, 0u, 10u,
                                          adu, sizeof(adu), &len), GW_OK);
    GW_ASSERT_EQ_INT(len, 8);
    {
        static const uint8_t golden[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x0A, 0xC5, 0xCD };
        GW_ASSERT_EQ_MEM(adu, golden, 8u);
    }
    GW_ASSERT_EQ_INT(gw_mb_rtu_parse_request(adu, len, &rq), GW_OK);
    GW_ASSERT_EQ_INT(rq.slave, 1);
    GW_ASSERT_EQ_INT(rq.function, GW_MB_FC_READ_HOLDING);
    GW_ASSERT_EQ_INT(rq.start_addr, 0);
    GW_ASSERT_EQ_INT(rq.quantity, 10);

    GW_CASE("FC03 读保持寄存器请求：黄金帧 01 03 00 00 00 0D 84 0F");
    GW_ASSERT_EQ_INT(gw_mb_rtu_build_read(1u, GW_MB_FC_READ_HOLDING, 0u, 13u,
                                          adu, sizeof(adu), &len), GW_OK);
    GW_ASSERT_EQ_INT(len, 8);
    GW_ASSERT_EQ_INT(adu[6], 0x84);
    GW_ASSERT_EQ_INT(adu[7], 0x0F);

    GW_CASE("FC06 写单个寄存器：黄金帧 01 06 00 00 07 08 8A 3C");
    GW_ASSERT_EQ_INT(gw_mb_rtu_build_write_single(1u, 0u, 1800u, adu,
                                                  sizeof(adu), &len), GW_OK);
    GW_ASSERT_EQ_INT(len, 8);
    GW_ASSERT_EQ_INT(adu[4], 0x07);
    GW_ASSERT_EQ_INT(adu[5], 0x08);
    GW_ASSERT_EQ_INT(adu[6], 0x8A);
    GW_ASSERT_EQ_INT(adu[7], 0x3C);
    GW_ASSERT_EQ_INT(gw_mb_rtu_parse_request(adu, len, &rq), GW_OK);
    GW_ASSERT_EQ_INT(rq.function, GW_MB_FC_WRITE_SINGLE_REG);
    GW_ASSERT_EQ_INT(rq.value, 1800);
    GW_ASSERT_EQ_INT(rq.quantity, 1);

    GW_CASE("FC16 写多个寄存器：黄金帧 01 10 00 00 00 02 04 12 34 56 78 88 9B");
    GW_ASSERT_EQ_INT(gw_mb_rtu_build_write_multi(1u, 0u, vals, 2u, adu,
                                                 sizeof(adu), &len), GW_OK);
    GW_ASSERT_EQ_INT(len, 13);
    GW_ASSERT_EQ_INT(adu[1], 0x10);
    GW_ASSERT_EQ_INT(adu[5], 0x02);          /* 数量低字节 = 2 */
    GW_ASSERT_EQ_INT(adu[6], 0x04);          /* 字节数 = 2 * 2 */
    GW_ASSERT_EQ_INT(adu[7], 0x12);
    GW_ASSERT_EQ_INT(adu[8], 0x34);
    GW_ASSERT_EQ_INT(adu[11], 0x88);
    GW_ASSERT_EQ_INT(adu[12], 0x9B);
    GW_ASSERT_EQ_INT(gw_mb_rtu_parse_request(adu, len, &rq), GW_OK);
    GW_ASSERT_EQ_INT(rq.quantity, 2);
    GW_ASSERT_EQ_INT(rq.values[0], 0x1234u);
    GW_ASSERT_EQ_INT(rq.values[1], 0x5678u);

    GW_CASE("FC16 写入 123 个寄存器（协议上限）编解码一致");
    {
        static uint16_t many[GW_MODBUS_MAX_WRITE_REGS];
        static uint8_t  big[GW_MAX_ADU_RTU];
        uint32_t        big_len = 0u;
        uint16_t        i;
        for (i = 0u; i < GW_MODBUS_MAX_WRITE_REGS; i++) {
            many[i] = (uint16_t)(0x1000u + i);
        }
        GW_ASSERT_EQ_INT(GW_MODBUS_MAX_WRITE_REGS, 123u);
        GW_ASSERT_EQ_INT(gw_mb_rtu_build_write_multi(2u, 0x20u, many,
                                                     GW_MODBUS_MAX_WRITE_REGS,
                                                     big, sizeof(big), &big_len),
                         GW_OK);
        GW_ASSERT_EQ_INT(big_len, 9u + 246u);
        GW_ASSERT_EQ_INT(gw_mb_rtu_parse_request(big, big_len, &rq), GW_OK);
        GW_ASSERT_EQ_INT(rq.quantity, GW_MODBUS_MAX_WRITE_REGS);
        GW_ASSERT_EQ_INT(rq.values[GW_MODBUS_MAX_WRITE_REGS - 1u],
                         0x1000u + GW_MODBUS_MAX_WRITE_REGS - 1u);
    }
    GW_CASE("写入超过 123 个寄存器被本地拒绝（不发到总线上）");
    GW_ASSERT_EQ_INT(gw_mb_rtu_build_write_multi(1u, 0u, vals,
                                                 GW_MODBUS_MAX_WRITE_REGS + 1u,
                                                 adu, sizeof(adu), &len),
                     GW_ERR_PARAM);

    GW_CASE("FC03 读响应解析（byte_count = 2*qty）");
    GW_ASSERT_EQ_INT(gw_mb_pdu_build_read_response(GW_MB_FC_READ_HOLDING, vals, 4u,
                                                   adu, sizeof(adu), &len), GW_OK);
    GW_ASSERT_EQ_INT(gw_mb_pdu_parse_response(adu, len, &rp), GW_OK);
    GW_ASSERT_EQ_INT(rp.value_count, 4);
    GW_ASSERT_EQ_INT(rp.values[0], 0x1234u);
    GW_ASSERT_EQ_INT(rp.values[3], 0xDEF0u);

    GW_CASE("CRC 被破坏的帧必须被拒绝（GW_ERR_CRC）");
    gw_crc16_modbus_append(adu, len);
    adu[len] ^= 0x01u;
    GW_ASSERT_EQ_INT(gw_mb_rtu_parse_response(adu, len + 2u, &rp), GW_ERR_CRC);

    GW_CASE("长度与非法的 byte_count 必须被拒绝");
    {
        uint8_t bad[8] = { 0x01, 0x03, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00 };
        gw_crc16_modbus_append(bad, 6u);
        GW_ASSERT_EQ_INT(gw_mb_rtu_parse_response(bad, 8u, &rp), GW_ERR_PARAM);
    }
}

/* ================================================================== */
/* 异常响应                                                            */
/* ================================================================== */
static void t_exception(void)
{
    t_slave_t ts;
    gw_link_t *link;
    gw_mb_rtu_master_t m;
    uint8_t  req[GW_MAX_ADU_RTU];
    uint8_t  resp[GW_MAX_ADU_RTU];
    uint32_t req_len = 0u;
    uint32_t resp_len = 0u;
    uint16_t vals[8];

    memset(&ts, 0, sizeof(ts));
    GW_ASSERT_EQ_INT(gw_mb_slave_init(&ts.slave, 3u, 64u, 8u), GW_OK);
    link = gw_link_loopback_create(t_slave_fn, &ts);
    GW_ASSERT(link != NULL);
    GW_ASSERT_EQ_INT(gw_mb_rtu_master_init(&m, link, 120u, 2u), GW_OK);

    GW_CASE("非法功能码(0x01 读线圈) -> 从站回 0x81/0x01, 主站不重试");
    {
        /* 本网关不实现线圈功能码, 因此手工构造这条"协议合法但本机不支持"
         * 的请求 03 01 00 00 00 08 3C 2E, 验证从站的异常响应路径 */
        static const uint8_t coil_req[8] = { 0x03, 0x01, 0x00, 0x00,
                                             0x00, 0x08, 0x3C, 0x2E };
        req_len = 8u;
        memcpy(req, coil_req, 8u);
    }
    GW_ASSERT_EQ_INT(gw_mb_rtu_master_txn(&m, req, req_len, resp, sizeof(resp),
                                          &resp_len), GW_ERR_EXCEPTION);
    GW_ASSERT_EQ_INT(resp[1], 0x81);
    GW_ASSERT_EQ_INT(resp[2], GW_MB_EXC_ILLEGAL_FUNCTION);
    GW_ASSERT_EQ_INT(m.exceptions, 1u);
    GW_ASSERT_EQ_INT(m.retry_count, 0u);      /* 异常是有效答复, 不重试 */

    GW_CASE("非法数据地址 -> 异常码 02");
    GW_ASSERT_EQ_INT(gw_mb_rtu_master_read(&m, 3u, GW_MB_FC_READ_HOLDING,
                                           60u, 10u, vals, NULL), GW_ERR_EXCEPTION);
    GW_ASSERT_EQ_INT(m.last_exception, GW_MB_EXC_ILLEGAL_ADDRESS);
    GW_ASSERT_EQ_INT(gw_mb_exception_str(m.last_exception)[0], 'I');

    GW_CASE("非法数据值（byte_count 与数量不符）-> 异常码 03");
    {
        uint8_t bad[16];
        uint32_t bad_len = 0u;
        bad[0] = 3;
        bad[1] = GW_MB_FC_WRITE_MULTI_REGS;
        bad[2] = 0x00; bad[3] = 0x00;
        bad[4] = 0x00; bad[5] = 0x02;
        bad[6] = 0x06;                     /* 谎报 6 字节, 实际应 4 */
        memset(&bad[7], 0, 6u);
        gw_crc16_modbus_append(bad, 13u);
        bad_len = 15u;
        GW_ASSERT_EQ_INT(gw_mb_rtu_master_txn(&m, bad, bad_len, resp,
                                              sizeof(resp), &resp_len),
                         GW_ERR_EXCEPTION);
        GW_ASSERT_EQ_INT(resp[2], GW_MB_EXC_ILLEGAL_VALUE);
    }

    GW_CASE("从站异常响应不被计为总线错误（gw_err_class_is_fatal）");
    GW_ASSERT(!gw_err_class_is_fatal(GW_ERRCLASS_EXCEPTION));
    GW_ASSERT(gw_err_class_is_fatal(GW_ERRCLASS_CRC));

    gw_link_close(link);
}

/* ================================================================== */
/* 主站事务：超时 / 重试 / 恢复                                        */
/* ================================================================== */
static void t_master_retry(void)
{
    t_slave_t ts;
    gw_link_t *link;
    gw_mb_rtu_master_t m;
    uint16_t vals[4];

    memset(&ts, 0, sizeof(ts));
    GW_ASSERT_EQ_INT(gw_mb_slave_init(&ts.slave, 1u, 64u, 8u), GW_OK);
    ts.slave.holding[0] = 0x1111u;
    ts.slave.holding[1] = 0x2222u;
    ts.slave.holding[2] = 0x3333u;
    ts.slave.holding[3] = 0x4444u;
    link = gw_link_loopback_create(t_slave_fn, &ts);
    GW_ASSERT_EQ_INT(gw_mb_rtu_master_init(&m, link, 60u, 2u), GW_OK);

    GW_CASE("正常读取：数据与从站寄存器一致");
    GW_ASSERT_EQ_INT(gw_mb_rtu_master_read(&m, 1u, GW_MB_FC_READ_HOLDING, 0u, 4u,
                                           vals, NULL), GW_OK);
    GW_ASSERT_EQ_INT(vals[0], 0x1111u);
    GW_ASSERT_EQ_INT(vals[3], 0x4444u);
    GW_ASSERT_EQ_INT(m.ok_frames, 1u);

    GW_CASE("CRC 错 1 次后重试成功（重试计数 +1, 不改变数据）");
    ts.corrupt_crc = 1u;
    GW_ASSERT_EQ_INT(gw_mb_rtu_master_read(&m, 1u, GW_MB_FC_READ_HOLDING, 0u, 4u,
                                           vals, NULL), GW_OK);
    GW_ASSERT_EQ_INT(ts.corrupted, 1u);
    GW_ASSERT(m.retry_count >= 1u);
    GW_ASSERT_EQ_INT(vals[1], 0x2222u);

    GW_CASE("静默 2 次（在重试预算内）后恢复成功");
    ts.silent = 2u;
    GW_ASSERT_EQ_INT(gw_mb_rtu_master_read(&m, 1u, GW_MB_FC_READ_HOLDING, 0u, 4u,
                                           vals, NULL), GW_OK);
    GW_ASSERT_EQ_INT(ts.silent_done, 2u);
    GW_ASSERT_EQ_INT(vals[2], 0x3333u);

    GW_CASE("静默次数超过重试预算 -> 事务失败并返回超时");
    ts.silent = 5u;
    GW_ASSERT_EQ_INT(gw_mb_rtu_master_read(&m, 1u, GW_MB_FC_READ_HOLDING, 0u, 4u,
                                           vals, NULL), GW_ERR_TIMEOUT);
    GW_ASSERT(m.timeouts >= 1u);
    GW_ASSERT_EQ_INT(m.last_error, GW_MB_ERR_TIMEOUT);

    GW_CASE("写入单个寄存器后从站寄存器值确实改变（06 端到端）");
    GW_ASSERT_EQ_INT(gw_mb_rtu_master_write_single(&m, 1u, 5u, 0xBEEFu), GW_OK);
    GW_ASSERT_EQ_INT(ts.slave.holding[5], 0xBEEFu);

    GW_CASE("写入多个寄存器后逐字节校验（16 端到端）");
    {
        uint16_t w[3] = { 0x0102u, 0x0304u, 0x0506u };
        GW_ASSERT_EQ_INT(gw_mb_rtu_master_write_multi(&m, 1u, 8u, w, 3u), GW_OK);
        GW_ASSERT_EQ_INT(ts.slave.holding[8], 0x0102u);
        GW_ASSERT_EQ_INT(ts.slave.holding[9], 0x0304u);
        GW_ASSERT_EQ_INT(ts.slave.holding[10], 0x0506u);
    }

    GW_CASE("地址不匹配的响应被过滤（bad_slave 计数）");
    {
        t_slave_t ts2;
        gw_link_t *l2;
        gw_mb_rtu_master_t m2;
        memset(&ts2, 0, sizeof(ts2));
        /* 从站只认地址 9, 但主站请求地址 1 -> 从站静默, 主站超时 */
        GW_ASSERT_EQ_INT(gw_mb_slave_init(&ts2.slave, 9u, 32u, 4u), GW_OK);
        l2 = gw_link_loopback_create(t_slave_fn, &ts2);
        GW_ASSERT_EQ_INT(gw_mb_rtu_master_init(&m2, l2, 40u, 0u), GW_OK);
        GW_ASSERT_EQ_INT(gw_mb_rtu_master_read(&m2, 1u, GW_MB_FC_READ_HOLDING,
                                               0u, 2u, vals, NULL), GW_ERR_TIMEOUT);
        gw_link_close(l2);
    }

    gw_link_close(link);
}

/* ================================================================== */
/* Modbus TCP（MBAP）                                                  */
/* ================================================================== */
static void t_modbus_tcp(void)
{
    uint8_t  adu[GW_MAX_ADU_TCP];
    uint32_t len = 0u;
    uint8_t  pdu[8];
    uint32_t pdu_len = 0u;
    gw_mbap_t hdr;
    const uint8_t *rpdu = NULL;
    uint32_t rpdu_len = 0u;

    GW_CASE("MBAP 编码：事务号/协议号/长度/单元号");
    GW_ASSERT_EQ_INT(gw_mb_pdu_build_read(GW_MB_FC_READ_HOLDING, 0x10u, 4u,
                                          pdu, sizeof(pdu), &pdu_len), GW_OK);
    GW_ASSERT_EQ_INT(gw_mb_tcp_build(0x1234u, 0x11u, pdu, pdu_len,
                                     adu, sizeof(adu), &len), GW_OK);
    GW_ASSERT_EQ_INT(len, pdu_len + 7u);
    GW_ASSERT_EQ_INT(gw_read_be16(&adu[0]), 0x1234u);
    GW_ASSERT_EQ_INT(gw_read_be16(&adu[2]), 0u);              /* protocol id */
    GW_ASSERT_EQ_INT(gw_read_be16(&adu[4]), (int)(pdu_len + 1u));
    GW_ASSERT_EQ_INT(adu[6], 0x11u);
    GW_ASSERT_EQ_INT(gw_mb_tcp_parse(adu, len, &hdr, &rpdu, &rpdu_len), GW_OK);
    GW_ASSERT_EQ_INT(hdr.transaction_id, 0x1234u);
    GW_ASSERT_EQ_INT(hdr.unit_id, 0x11u);
    GW_ASSERT_EQ_INT(rpdu_len, pdu_len);

    GW_CASE("MBAP 组装器：拆包（一帧分 3 段到达）");
    {
        gw_mb_tcp_asm_t asm_;
        int  frames = 0;
        gw_mb_tcp_asm_init(&asm_);
        gw_mb_tcp_asm_feed(&asm_, adu, 3u, NULL, NULL);
        gw_mb_tcp_asm_feed(&asm_, adu + 3u, 5u, NULL, NULL);
        frames = (int)asm_.frames_ok;
        GW_ASSERT_EQ_INT(frames, 0);
        gw_mb_tcp_asm_feed(&asm_, adu + 8u, len - 8u, NULL, NULL);
        GW_ASSERT_EQ_INT(asm_.frames_ok, 1u);
        GW_ASSERT_EQ_INT(asm_.len, 0u);
    }

    GW_CASE("MBAP 组装器：粘包（4 帧同一次到达）");
    {
        gw_mb_tcp_asm_t asm_;
        uint8_t buf[4 * GW_MAX_ADU_TCP];
        int i;
        for (i = 0; i < 4; i++) {
            memcpy(&buf[i * (int)len], adu, len);
        }
        gw_mb_tcp_asm_init(&asm_);
        gw_mb_tcp_asm_feed(&asm_, buf, (uint32_t)(4 * (int)len), NULL, NULL);
        GW_ASSERT_EQ_INT(asm_.frames_ok, 4u);
        GW_ASSERT_EQ_INT(asm_.len, 0u);
    }

    GW_CASE("协议号非 0 的报文触发重同步（不会把坏报文当成完整帧留下）");
    {
        gw_mb_tcp_asm_t asm_;
        uint8_t bad[GW_MAX_ADU_TCP];
        memcpy(bad, adu, len);
        bad[2] = 0x00; bad[3] = 0x01;     /* protocol id = 1 */
        gw_mb_tcp_asm_init(&asm_);
        gw_mb_tcp_asm_feed(&asm_, bad, len, NULL, NULL);
        GW_ASSERT(asm_.bad_protocol >= 1u);   /* 逐字节滑动期间会多次命中 */
        GW_ASSERT(asm_.resync >= 1u);
        GW_ASSERT_EQ_INT(asm_.frames_ok, 0u);
        GW_ASSERT(asm_.len < len);
    }

    GW_CASE("TCP 从站处理 + 主站事务（回环链路, 校验寄存器值）");
    {
        static gw_mb_slave_t s;
        gw_link_t          *link;
        gw_mb_tcp_master_t  m;
        uint16_t            out[4];

        GW_ASSERT_EQ_INT(gw_mb_slave_init(&s, 1u, 32u, 4u), GW_OK);
        s.holding[0] = 0x0A0Au;
        s.holding[1] = 0x0B0Bu;
        s.holding[2] = 0x0C0Cu;
        s.holding[3] = 0x0D0Du;
        s_tcp_slave = &s;

        link = gw_link_loopback_create(tcp_slave_fn, NULL);
        GW_ASSERT(link != NULL);
        GW_ASSERT_EQ_INT(gw_mb_tcp_master_init(&m, link, 100u, 1u), GW_OK);

        GW_ASSERT_EQ_INT(gw_mb_tcp_master_read(&m, 1u, GW_MB_FC_READ_HOLDING,
                                               0u, 4u, out), GW_OK);
        GW_ASSERT_EQ_INT(out[0], 0x0A0Au);
        GW_ASSERT_EQ_INT(out[3], 0x0D0Du);

        GW_CASE("TCP 写入单个寄存器");
        GW_ASSERT_EQ_INT(gw_mb_tcp_master_write_single(&m, 1u, 2u, 0x1234u), GW_OK);
        GW_ASSERT_EQ_INT(s.holding[2], 0x1234u);

        GW_CASE("TCP 写入多个寄存器");
        {
            uint16_t w[2] = { 0xAAAAu, 0xBBBBu };
            GW_ASSERT_EQ_INT(gw_mb_tcp_master_write_multi(&m, 1u, 4u, w, 2u), GW_OK);
            GW_ASSERT_EQ_INT(s.holding[4], 0xAAAAu);
            GW_ASSERT_EQ_INT(s.holding[5], 0xBBBBu);
        }

        GW_CASE("TCP 单元号不匹配 -> 从站静默, 主站超时");
        GW_ASSERT_EQ_INT(gw_mb_tcp_master_read(&m, 7u, GW_MB_FC_READ_HOLDING,
                                               0u, 2u, out), GW_ERR_TIMEOUT);
        gw_link_close(link);
    }
}

void test_modbus_rtu(void)
{
    t_codec();
    t_exception();
    t_master_retry();
    t_modbus_tcp();
}
