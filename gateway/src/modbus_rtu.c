/*
 * modbus_rtu.c - Modbus RTU 主站事务与从站模拟
 *
 * 超时与重试策略（简历"主从站轮询与超时重试"的落地）：
 *   - 单次响应超时 200ms（9600bps 下一帧 8 字节约 8ms, 200ms 足够
 *     覆盖从站扫描周期 + 总线仲裁, 又不会让轮询周期失控）；
 *   - 超时/CRC 错重试 2 次, 每次重试前插入 2ms 静默, 保证帧边界清晰；
 *   - 从站明确返回异常响应时不重试：异常是"有效答复", 重试只会
 *     浪费总线时间, 正确做法是立刻上报并交由服务层决策。
 *
 * 每次事务后把收发质量（CRC 错 / 残帧 / 重同步次数）留给上层,
 * 由 bus_health 模块做分级与隔离判决。
 */
#include "modbus_rtu.h"
#include "gw_crc.h"
#include "gw_log.h"
#include "port_rtos.h"

#include <string.h>

/* ================================================================== */
/* 期望长度推断                                                        */
/* ================================================================== */
uint32_t gw_mb_rtu_expected_len(const uint8_t *buf, uint32_t len)
{
    uint8_t fc;

    if ((buf == NULL) || (len < 2u)) {
        return 0u;
    }
    fc = buf[1];

    if (gw_mb_is_exception_function(fc)) {
        return 5u;                         /* 地址 + FC + 异常码 + CRC */
    }

    switch (fc) {
    case GW_MB_FC_READ_COILS:
    case GW_MB_FC_READ_DISCRETE:
    case GW_MB_FC_READ_HOLDING:
    case GW_MB_FC_READ_INPUT: {
        /*
         * 同一个功能码, 请求固定 8 字节, 响应为 5 + ByteCount。
         * 在字节流里我们无法先验地知道"方向", 因此返回**两个候选中较大
         * 的那个**: 这保证"响应还在累积中"时不会被误判成错帧
         * （早期版本返回请求长度 8, 导致第 8 个字节就报 CRC 错,
         *   整条读响应永远收不全 —— 这正是混线现场最难查的一类 bug）。
         * 真正的判帧由"每个字节都做一次 CRC 校验"完成, 这里只用于
         * 错误分类与超长保护。
         */
        uint32_t cand = 8u;
        if (len >= 3u) {
            uint32_t resp = 5u + (uint32_t)buf[2];
            if ((resp <= GW_MAX_ADU_RTU) && (resp > cand)) {
                cand = resp;
            }
        }
        return cand;
    }

    case GW_MB_FC_WRITE_SINGLE_REG:
    case GW_MB_FC_WRITE_SINGLE_COIL:
        return 8u;                         /* 请求与响应同为 8 字节 */

    case GW_MB_FC_WRITE_MULTI_REGS:
    case GW_MB_FC_WRITE_MULTI_COILS: {
        uint32_t cand = 8u;                /* 响应固定 8 字节 */
        if (len >= 7u) {
            uint32_t req = 9u + (uint32_t)buf[6];
            if ((req <= GW_MAX_ADU_RTU) && (req > cand)) {
                cand = req;
            }
        }
        return cand;
    }

    default:
        return 0u;                         /* 未知功能码：交给 CRC 判据 */
    }
}

/* ================================================================== */
/* ADU 编解码                                                          */
/* ================================================================== */
static int rtu_wrap_adu(uint8_t slave, const uint8_t *pdu, uint32_t pdu_len,
                        uint8_t *out, uint32_t cap, uint32_t *out_len)
{
    if ((pdu == NULL) || (out == NULL) || (out_len == NULL)) {
        return GW_ERR_PARAM;
    }
    if (cap < pdu_len + 3u) {
        return GW_ERR_PARAM;
    }
    if (pdu_len > GW_MODBUS_MAX_PDU) {
        return GW_ERR_PARAM;
    }

    out[0] = slave;
    memcpy(&out[1], pdu, pdu_len);
    gw_crc16_modbus_append(out, pdu_len + 1u);
    *out_len = pdu_len + 3u;
    return GW_OK;
}

int gw_mb_rtu_build_read(uint8_t slave, uint8_t fc, uint16_t addr,
                         uint16_t qty, uint8_t *out, uint32_t cap,
                         uint32_t *out_len)
{
    uint8_t pdu[8];
    uint32_t pdu_len = 0u;
    int rc;

    rc = gw_mb_pdu_build_read(fc, addr, qty, pdu, sizeof(pdu), &pdu_len);
    if (rc != GW_OK) {
        return rc;
    }
    return rtu_wrap_adu(slave, pdu, pdu_len, out, cap, out_len);
}

int gw_mb_rtu_build_write_single(uint8_t slave, uint16_t addr, uint16_t value,
                                 uint8_t *out, uint32_t cap, uint32_t *out_len)
{
    uint8_t pdu[8];
    uint32_t pdu_len = 0u;
    int rc;

    rc = gw_mb_pdu_build_write_single(addr, value, pdu, sizeof(pdu), &pdu_len);
    if (rc != GW_OK) {
        return rc;
    }
    return rtu_wrap_adu(slave, pdu, pdu_len, out, cap, out_len);
}

int gw_mb_rtu_build_write_multi(uint8_t slave, uint16_t addr,
                                const uint16_t *values, uint16_t qty,
                                uint8_t *out, uint32_t cap, uint32_t *out_len)
{
    uint8_t  pdu[GW_MAX_ADU_RTU];
    uint32_t pdu_len = 0u;
    int      rc;

    rc = gw_mb_pdu_build_write_multi(addr, values, qty, pdu, sizeof(pdu), &pdu_len);
    if (rc != GW_OK) {
        return rc;
    }
    return rtu_wrap_adu(slave, pdu, pdu_len, out, cap, out_len);
}

int gw_mb_rtu_parse_request(const uint8_t *adu, uint32_t len,
                            gw_mb_request_t *out)
{
    int rc;

    if ((adu == NULL) || (out == NULL)) {
        return GW_ERR_PARAM;
    }
    if ((len < 4u) || (len > GW_MAX_ADU_RTU)) {
        return GW_ERR_PARAM;
    }
    if (!gw_crc16_modbus_check_frame(adu, len)) {
        return GW_ERR_CRC;
    }

    rc = gw_mb_pdu_parse_request(&adu[1], len - 3u, out);
    if (rc != GW_OK) {
        return rc;
    }
    out->slave = adu[0];
    return GW_OK;
}

int gw_mb_rtu_parse_response(const uint8_t *adu, uint32_t len,
                             gw_mb_response_t *out)
{
    int rc;

    if ((adu == NULL) || (out == NULL)) {
        return GW_ERR_PARAM;
    }
    if ((len < 4u) || (len > GW_MAX_ADU_RTU)) {
        return GW_ERR_PARAM;
    }
    if (!gw_crc16_modbus_check_frame(adu, len)) {
        return GW_ERR_CRC;
    }

    rc = gw_mb_pdu_parse_response(&adu[1], len - 3u, out);
    out->slave = adu[0];
    return rc;   /* 异常响应时返回 GW_ERR_EXCEPTION, 但结构体已填好 */
}

/* ================================================================== */
/* 主站                                                                */
/* ================================================================== */
static void rtu_rx_cb(void *ctx, const gw_fs_event_t *ev)
{
    gw_mb_rtu_master_t *m = (gw_mb_rtu_master_t *)ctx;

    switch (ev->kind) {
    case GW_FS_EV_FRAME:
        if (ev->len <= sizeof(m->rx.frame)) {
            memcpy(m->rx.frame, ev->data, ev->len);
            m->rx.frame_len = ev->len;
            m->rx.got_frame = true;
            m->rx_frames++;
        }
        break;
    case GW_FS_EV_CRC_ERROR:
        m->rx.crc_errors++;
        m->crc_errors++;
        break;
    case GW_FS_EV_SHORT_FRAME:
        m->rx.short_frames++;
        m->short_frames++;
        break;
    case GW_FS_EV_RESYNC:
        m->rx.resync_count++;
        break;
    case GW_FS_EV_OVERSIZE:
        m->crc_errors++;
        break;
    default:
        break;
    }
}

int gw_mb_rtu_master_init(gw_mb_rtu_master_t *m, gw_link_t *link,
                          uint32_t timeout_ms, uint32_t retries)
{
    if ((m == NULL) || (link == NULL)) {
        return GW_ERR_PARAM;
    }
    memset(m, 0, sizeof(*m));
    m->link       = link;
    m->timeout_ms = (timeout_ms == 0u) ? GW_MB_RESP_TIMEOUT_MS : timeout_ms;
    m->retries    = retries;

    return gw_frame_sync_init(&m->rx.fs, m->rx.buf, sizeof(m->rx.buf),
                              GW_T35_US, gw_mb_rtu_expected_len,
                              rtu_rx_cb, m);
}

void gw_mb_rtu_master_set_frame_source(gw_mb_rtu_master_t *m,
                                       int (*fn)(void *ctx, uint8_t *frame,
                                                 uint32_t cap, uint32_t *len,
                                                 uint32_t timeout_ms),
                                       void *ctx)
{
    if (m != NULL) {
        m->frame_source     = fn;
        m->frame_source_ctx = ctx;
    }
}

/* 收一帧：返回 GW_OK / GW_ERR_TIMEOUT / GW_ERR_CRC / GW_ERR_IO */
static int rtu_wait_frame(gw_mb_rtu_master_t *m, uint32_t timeout_ms)
{
    uint8_t  chunk[GW_MB_RX_CHUNK];
    uint32_t deadline;
    uint32_t crc_before = m->crc_errors;
    uint32_t short_before = m->short_frames;

    m->rx.got_frame = false;
    m->rx.frame_len = 0u;

    /* --- 路径 A：外部帧来源（DMA + 环形缓冲 + 帧同步, 目标板/整机仿真） --- */
    if (m->frame_source != NULL) {
        uint32_t len = 0u;
        int      rc = m->frame_source(m->frame_source_ctx, m->rx.frame,
                                      (uint32_t)sizeof(m->rx.frame), &len,
                                      timeout_ms);
        if (rc == GW_OK) {
            m->rx.frame_len = len;
            m->rx.got_frame = true;
            m->rx_frames++;
            return GW_OK;
        }
        m->source_timeouts++;
        if ((m->crc_errors > crc_before) || (m->short_frames > short_before)) {
            return GW_ERR_CRC;
        }
        m->timeouts++;
        return GW_ERR_TIMEOUT;
    }

    /* --- 路径 B：主站自己收字节并做帧同步（PC 单元测试路径） --- */
    gw_frame_sync_reset(&m->rx.fs);
    deadline = gw_port_tick_ms() + timeout_ms;

    for (;;) {
        int32_t remain = (int32_t)(deadline - gw_port_tick_ms());
        int     n;

        if (remain <= 0) {
            gw_frame_sync_tick(&m->rx.fs, gw_port_now_us());
            if (m->rx.got_frame) {
                break;
            }
            if (m->crc_errors > crc_before) {
                return GW_ERR_CRC;         /* 收到过 CRC 错的帧, 立即重试 */
            }
            if (m->short_frames > short_before) {
                return GW_ERR_CRC;
            }
            m->timeouts++;
            return GW_ERR_TIMEOUT;
        }

        n = gw_link_recv(m->link, chunk, sizeof(chunk), remain);
        if (n > 0) {
            gw_frame_sync_feed(&m->rx.fs, chunk, (uint32_t)n, gw_port_now_us());
            if (m->rx.got_frame) {
                break;
            }
            /* CRC 错 / 残帧：不必傻等到超时, 立刻让上层重试, 缩短恢复时间 */
            if ((m->crc_errors > crc_before) || (m->short_frames > short_before)) {
                return GW_ERR_CRC;
            }
            continue;
        }
        if (n == GW_ERR_TIMEOUT) {
            continue;                      /* 由 deadline 统一裁决 */
        }
        if (n == GW_ERR_EMPTY) {
            continue;
        }
        return GW_ERR_IO;
    }

    return GW_OK;
}

int gw_mb_rtu_master_txn(gw_mb_rtu_master_t *m, const uint8_t *req,
                         uint32_t req_len, uint8_t *resp, uint32_t cap,
                         uint32_t *resp_len)
{
    uint32_t attempt;
    int      last = GW_ERR_TIMEOUT;

    if ((m == NULL) || (req == NULL) || (resp == NULL) || (resp_len == NULL)) {
        return GW_ERR_PARAM;
    }

    for (attempt = 0u; attempt <= m->retries; attempt++) {
        int n;
        int rc;

        m->last_attempts = attempt + 1u;

        if (attempt > 0u) {
            m->retry_count++;
            GW_LOGW("MB", "retry %u/%u after %s", (unsigned)attempt,
                    (unsigned)m->retries, gw_status_str(last));
            gw_port_delay_ms(GW_MB_INTER_FRAME_MS);
        }

        m->tx_frames++;
        n = gw_link_send(m->link, req, req_len, (int32_t)m->timeout_ms);
        if (n < 0) {
            m->last_error = GW_MB_ERR_LINK;
            return GW_ERR_IO;
        }

        rc = rtu_wait_frame(m, m->timeout_ms);
        if (rc != GW_OK) {
            last = rc;
            m->last_error = (rc == GW_ERR_TIMEOUT) ? GW_MB_ERR_TIMEOUT
                                                   : GW_MB_ERR_BAD_CRC;
            continue;
        }

        /* 地址过滤：总线上可能挂着别的从站 */
        if (m->rx.frame[0] != req[0]) {
            m->bad_slave++;
            last = GW_ERR_NOT_FOUND;
            m->last_error = GW_MB_ERR_BAD_SLAVE;
            GW_LOGW("MB", "slave mismatch: got %u, want %u",
                    (unsigned)m->rx.frame[0], (unsigned)req[0]);
            continue;
        }

        /* 功能码校验：异常响应的高位置 1 也算匹配 */
        if ((m->rx.frame[1] != req[1]) &&
            (m->rx.frame[1] != (uint8_t)(req[1] | GW_MB_FC_EXCEPTION_MASK))) {
            m->last_error = GW_MB_ERR_BAD_FUNCTION;
            last = GW_ERR_STATE;
            continue;
        }

        if (m->rx.frame_len > cap) {
            return GW_ERR_OVERFLOW;
        }
        memcpy(resp, m->rx.frame, m->rx.frame_len);
        *resp_len = m->rx.frame_len;

        if (gw_mb_is_exception_function(m->rx.frame[1])) {
            m->exceptions++;
            m->last_exception = m->rx.frame[2];
            m->last_error     = GW_MB_ERR_EXCEPTION;
            GW_LOGW("MB", "slave %u exception fc=0x%02X code=0x%02X (%s)",
                    (unsigned)req[0], (unsigned)req[1],
                    (unsigned)m->last_exception,
                    gw_mb_exception_str(m->last_exception));
            return GW_ERR_EXCEPTION;       /* 不重试 */
        }

        m->ok_frames++;
        m->last_error = GW_MB_ERR_NONE;
        return GW_OK;
    }

    m->last_error = (last == GW_ERR_TIMEOUT) ? GW_MB_ERR_TIMEOUT
                                            : GW_MB_ERR_BAD_CRC;
    GW_LOGW("MB", "transaction failed after %u attempt(s): %s",
            (unsigned)(m->retries + 1u), gw_status_str(last));
    return last;
}

int gw_mb_rtu_master_read(gw_mb_rtu_master_t *m, uint8_t slave, uint8_t fc,
                          uint16_t addr, uint16_t qty, uint16_t *out,
                          uint16_t *out_qty)
{
    uint8_t  req[GW_MAX_ADU_RTU];
    uint8_t  resp[GW_MAX_ADU_RTU];
    uint32_t req_len = 0u;
    uint32_t resp_len = 0u;
    gw_mb_response_t pr;
    int rc;
    uint16_t i;
    uint16_t want;

    if ((m == NULL) || (out == NULL)) {
        return GW_ERR_PARAM;
    }

    rc = gw_mb_rtu_build_read(slave, fc, addr, qty, req, sizeof(req), &req_len);
    if (rc != GW_OK) {
        return rc;
    }

    rc = gw_mb_rtu_master_txn(m, req, req_len, resp, sizeof(resp), &resp_len);
    if (rc == GW_ERR_EXCEPTION) {
        return GW_ERR_EXCEPTION;
    }
    if (rc != GW_OK) {
        return rc;
    }

    rc = gw_mb_rtu_parse_response(resp, resp_len, &pr);
    if (rc != GW_OK) {
        m->last_error = GW_MB_ERR_BAD_LENGTH;
        return rc;
    }
    if (pr.is_exception) {
        m->last_exception = pr.exception_code;
        return GW_ERR_EXCEPTION;
    }
    if (pr.function != fc) {
        return GW_ERR_STATE;
    }

    /* 响应数据量必须与请求一致, 否则按协议错误处理（防止错帧串位） */
    want = (qty < pr.value_count) ? qty : pr.value_count;
    if (pr.value_count != qty) {
        GW_LOGW("MB", "slave %u returned %u regs, requested %u",
                (unsigned)slave, (unsigned)pr.value_count, (unsigned)qty);
        m->last_error = GW_MB_ERR_BAD_LENGTH;
        return GW_ERR_STATE;
    }
    for (i = 0u; i < want; i++) {
        out[i] = pr.values[i];
    }
    if (out_qty != NULL) {
        *out_qty = want;
    }
    return GW_OK;
}

int gw_mb_rtu_master_write_single(gw_mb_rtu_master_t *m, uint8_t slave,
                                  uint16_t addr, uint16_t value)
{
    uint8_t  req[GW_MAX_ADU_RTU];
    uint8_t  resp[GW_MAX_ADU_RTU];
    uint32_t req_len = 0u;
    uint32_t resp_len = 0u;
    gw_mb_response_t pr;
    int rc;

    if (m == NULL) {
        return GW_ERR_PARAM;
    }
    rc = gw_mb_rtu_build_write_single(slave, addr, value, req, sizeof(req), &req_len);
    if (rc != GW_OK) {
        return rc;
    }
    rc = gw_mb_rtu_master_txn(m, req, req_len, resp, sizeof(resp), &resp_len);
    if (rc == GW_ERR_EXCEPTION) {
        return GW_ERR_EXCEPTION;
    }
    if (rc != GW_OK) {
        return rc;
    }
    rc = gw_mb_rtu_parse_response(resp, resp_len, &pr);
    if (rc != GW_OK) {
        return rc;
    }
    if (pr.is_exception || (pr.start_addr != addr) || (pr.value != value)) {
        return GW_ERR_EXCEPTION;
    }
    return GW_OK;
}

int gw_mb_rtu_master_write_multi(gw_mb_rtu_master_t *m, uint8_t slave,
                                 uint16_t addr, const uint16_t *values,
                                 uint16_t qty)
{
    uint8_t  req[GW_MAX_ADU_RTU];
    uint8_t  resp[GW_MAX_ADU_RTU];
    uint32_t req_len = 0u;
    uint32_t resp_len = 0u;
    gw_mb_response_t pr;
    int rc;

    if ((m == NULL) || (values == NULL)) {
        return GW_ERR_PARAM;
    }
    rc = gw_mb_rtu_build_write_multi(slave, addr, values, qty, req, sizeof(req),
                                     &req_len);
    if (rc != GW_OK) {
        return rc;
    }
    rc = gw_mb_rtu_master_txn(m, req, req_len, resp, sizeof(resp), &resp_len);
    if (rc == GW_ERR_EXCEPTION) {
        return GW_ERR_EXCEPTION;
    }
    if (rc != GW_OK) {
        return rc;
    }
    rc = gw_mb_rtu_parse_response(resp, resp_len, &pr);
    if (rc != GW_OK) {
        return rc;
    }
    if (pr.is_exception || (pr.start_addr != addr) || (pr.quantity != qty)) {
        return GW_ERR_EXCEPTION;
    }
    return GW_OK;
}

/* ================================================================== */
/* 从站模拟                                                            */
/* ================================================================== */
int gw_mb_slave_init(gw_mb_slave_t *s, uint8_t addr, uint16_t holding_count,
                     uint16_t input_count)
{
    if (s == NULL) {
        return GW_ERR_PARAM;
    }
    memset(s, 0, sizeof(*s));
    s->addr          = addr;
    s->holding_count = (holding_count > GW_MB_HOLDING_REGS) ? GW_MB_HOLDING_REGS
                                                            : holding_count;
    s->input_count   = (input_count > GW_MB_INPUT_REGS) ? GW_MB_INPUT_REGS
                                                        : input_count;
    return GW_OK;
}

static int slave_reply_exception(gw_mb_slave_t *s, uint8_t slave_addr,
                                 uint8_t fc, uint8_t code, uint8_t *resp,
                                 uint32_t cap, uint32_t *resp_len)
{
    uint8_t  pdu[4];
    uint32_t pdu_len = 0u;
    int      rc;

    rc = gw_mb_pdu_build_exception(fc, code, pdu, sizeof(pdu), &pdu_len);
    if (rc != GW_OK) {
        return rc;
    }
    s->exceptions_sent++;
    rc = rtu_wrap_adu(slave_addr, pdu, pdu_len, resp, cap, resp_len);
    return (rc == GW_OK) ? GW_ERR_EXCEPTION : rc;
}

int gw_mb_slave_process(gw_mb_slave_t *s, const uint8_t *req, uint32_t req_len,
                        uint8_t *resp, uint32_t cap, uint32_t *resp_len)
{
    gw_mb_request_t  rq;
    gw_mb_response_t dummy;
    uint8_t  pdu[GW_MAX_ADU_RTU];
    uint32_t pdu_len = 0u;
    int      rc;

    if ((s == NULL) || (req == NULL) || (resp == NULL) || (resp_len == NULL)) {
        return GW_ERR_PARAM;
    }
    if ((req_len < 4u) || (req_len > GW_MAX_ADU_RTU)) {
        return GW_ERR_PARAM;
    }
    if (!gw_crc16_modbus_check_frame(req, req_len)) {
        s->crc_rejected++;
        return GW_ERR_CRC;                 /* 真实从站对 CRC 错的帧保持静默 */
    }
    if ((req[0] != s->addr) && (req[0] != 0u)) {
        return GW_ERR_NOT_FOUND;           /* 不是发给自己的, 静默 */
    }
    GW_UNUSED(dummy);

    s->requests_handled++;
    rc = gw_mb_pdu_parse_request(&req[1], req_len - 3u, &rq);
    if (rc == GW_ERR_UNSUPPORTED) {
        return slave_reply_exception(s, req[0], req[1],
                                     GW_MB_EXC_ILLEGAL_FUNCTION,
                                     resp, cap, resp_len);
    }
    if (rc != GW_OK) {
        /* 长度/数量非法：按"非法数据值"答复, 让主站能区分于超时 */
        return slave_reply_exception(s, req[0], req[1],
                                     GW_MB_EXC_ILLEGAL_VALUE,
                                     resp, cap, resp_len);
    }

    switch (rq.function) {
    case GW_MB_FC_READ_HOLDING:
    case GW_MB_FC_READ_INPUT: {
        const uint16_t *tbl;
        uint16_t        cnt;
        if (rq.function == GW_MB_FC_READ_HOLDING) {
            tbl = s->holding;
            cnt = s->holding_count;
        } else {
            tbl = s->input;
            cnt = s->input_count;
        }
        if (((uint32_t)rq.start_addr + rq.quantity) > cnt) {
            return slave_reply_exception(s, req[0], req[1],
                                         GW_MB_EXC_ILLEGAL_ADDRESS,
                                         resp, cap, resp_len);
        }
        rc = gw_mb_pdu_build_read_response(rq.function, &tbl[rq.start_addr],
                                           rq.quantity, pdu, sizeof(pdu), &pdu_len);
        break;
    }

    case GW_MB_FC_WRITE_SINGLE_REG:
        if (rq.start_addr >= s->holding_count) {
            return slave_reply_exception(s, req[0], req[1],
                                         GW_MB_EXC_ILLEGAL_ADDRESS,
                                         resp, cap, resp_len);
        }
        s->holding[rq.start_addr] = rq.value;
        rc = gw_mb_pdu_build_write_response(rq.function, rq.start_addr, rq.value,
                                            pdu, sizeof(pdu), &pdu_len);
        break;

    case GW_MB_FC_WRITE_MULTI_REGS: {
        uint16_t i;
        if (((uint32_t)rq.start_addr + rq.quantity) > s->holding_count) {
            return slave_reply_exception(s, req[0], req[1],
                                         GW_MB_EXC_ILLEGAL_ADDRESS,
                                         resp, cap, resp_len);
        }
        for (i = 0u; i < rq.quantity; i++) {
            s->holding[rq.start_addr + i] = rq.values[i];
        }
        rc = gw_mb_pdu_build_write_response(rq.function, rq.start_addr,
                                            rq.quantity, pdu, sizeof(pdu),
                                            &pdu_len);
        break;
    }

    default:
        return slave_reply_exception(s, req[0], req[1],
                                     GW_MB_EXC_ILLEGAL_FUNCTION,
                                     resp, cap, resp_len);
    }

    if (rc != GW_OK) {
        return rc;
    }
    return rtu_wrap_adu(req[0], pdu, pdu_len, resp, cap, resp_len);
}
