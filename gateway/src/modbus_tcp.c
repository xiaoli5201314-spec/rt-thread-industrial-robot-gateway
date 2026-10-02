/*
 * modbus_tcp.c - Modbus TCP 编解码、组装与主站
 */
#include "modbus_tcp.h"
#include "gw_log.h"
#include "port_rtos.h"

#include <string.h>

/* ================================================================== */
/* MBAP 编解码                                                         */
/* ================================================================== */
int gw_mb_tcp_build(uint16_t tid, uint8_t unit, const uint8_t *pdu,
                    uint32_t pdu_len, uint8_t *out, uint32_t cap,
                    uint32_t *out_len)
{
    if ((pdu == NULL) || (out == NULL) || (out_len == NULL)) {
        return GW_ERR_PARAM;
    }
    if ((pdu_len == 0u) || (pdu_len > GW_MODBUS_MAX_PDU)) {
        return GW_ERR_PARAM;
    }
    if (cap < pdu_len + GW_MBAP_LEN) {
        return GW_ERR_PARAM;
    }

    gw_write_be16(&out[0], tid);
    gw_write_be16(&out[2], 0u);                       /* protocol id = 0 */
    gw_write_be16(&out[4], (uint16_t)(pdu_len + 1u)); /* unit + pdu      */
    out[6] = unit;
    memcpy(&out[7], pdu, pdu_len);
    *out_len = pdu_len + GW_MBAP_LEN;
    return GW_OK;
}

int gw_mb_tcp_parse(const uint8_t *adu, uint32_t len, gw_mbap_t *hdr,
                    const uint8_t **pdu, uint32_t *pdu_len)
{
    if ((adu == NULL) || (hdr == NULL) || (pdu == NULL) || (pdu_len == NULL)) {
        return GW_ERR_PARAM;
    }
    if (len < GW_MBAP_LEN + 1u) {
        return GW_ERR_PARAM;
    }

    hdr->transaction_id = gw_read_be16(&adu[0]);
    hdr->protocol_id    = gw_read_be16(&adu[2]);
    hdr->length         = gw_read_be16(&adu[4]);
    hdr->unit_id        = adu[6];

    if (hdr->protocol_id != 0u) {
        return GW_ERR_UNSUPPORTED;
    }
    if (hdr->length < 2u) {
        return GW_ERR_PARAM;
    }
    if (len != 6u + (uint32_t)hdr->length) {
        return GW_ERR_PARAM;
    }

    *pdu     = &adu[GW_MBAP_LEN];
    *pdu_len = (uint32_t)hdr->length - 1u;
    return GW_OK;
}

/* ================================================================== */
/* 报文组装                                                            */
/* ================================================================== */
void gw_mb_tcp_asm_init(gw_mb_tcp_asm_t *a)
{
    if (a != NULL) {
        memset(a, 0, sizeof(*a));
    }
}

void gw_mb_tcp_asm_feed(gw_mb_tcp_asm_t *a, const uint8_t *data, uint32_t len,
                        gw_mb_tcp_frame_cb cb, void *ctx)
{
    uint32_t i;

    if ((a == NULL) || (data == NULL) || (len == 0u)) {
        return;
    }

    for (i = 0u; i < len; i++) {
        if (a->len >= sizeof(a->buf)) {
            a->oversize++;
            /* 缓冲满：丢最旧 1 字节重新对齐, 保证不会永久卡死 */
            memmove(a->buf, a->buf + 1u, sizeof(a->buf) - 1u);
            a->len = sizeof(a->buf) - 1u;
        }
        a->buf[a->len++] = data[i];
        a->bytes_in++;

        /* 长度字段足够后即可判定整帧边界（这就是 TCP 的定界方式） */
        for (;;) {
            uint16_t mlen;
            uint32_t total;

            if (a->len < GW_MBAP_LEN) {
                break;
            }
            if (gw_read_be16(&a->buf[2]) != 0u) {
                a->bad_protocol++;
                memmove(a->buf, a->buf + 1u, a->len - 1u);
                a->len--;
                a->resync++;
                continue;
            }
            mlen = gw_read_be16(&a->buf[4]);
            if ((mlen < 2u) || ((uint32_t)mlen + 6u > sizeof(a->buf))) {
                a->oversize++;
                memmove(a->buf, a->buf + 1u, a->len - 1u);
                a->len--;
                a->resync++;
                continue;
            }
            total = 6u + (uint32_t)mlen;
            if (a->len < total) {
                break;                 /* 半包：等更多数据 */
            }
            a->frames_ok++;
            if (cb != NULL) {
                cb(ctx, a->buf, total);
            }
            memmove(a->buf, a->buf + total, a->len - total);
            a->len -= total;
        }
    }
}

/* ================================================================== */
/* 主站                                                                */
/* ================================================================== */
static void tcp_frame_cb(void *ctx, const uint8_t *adu, uint32_t len)
{
    gw_mb_tcp_master_t *m = (gw_mb_tcp_master_t *)ctx;

    m->rx_frames++;
    if (len <= sizeof(m->frame)) {
        memcpy(m->frame, adu, len);
        m->frame_len = len;
        m->got_frame = true;
    }
}

int gw_mb_tcp_master_init(gw_mb_tcp_master_t *m, gw_link_t *link,
                          uint32_t timeout_ms, uint32_t retries)
{
    if ((m == NULL) || (link == NULL)) {
        return GW_ERR_PARAM;
    }
    memset(m, 0, sizeof(*m));
    m->link       = link;
    m->timeout_ms = (timeout_ms == 0u) ? GW_MB_RESP_TIMEOUT_MS : timeout_ms;
    m->retries    = retries;
    m->next_tid   = 1u;
    gw_mb_tcp_asm_init(&m->asm_);
    return GW_OK;
}

void gw_mb_tcp_master_set_frame_source(gw_mb_tcp_master_t *m,
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

/* 收一个完整 ADU 并校验事务号；返回 GW_OK / GW_ERR_TIMEOUT / GW_ERR_IO */
static int tcp_wait_frame(gw_mb_tcp_master_t *m, uint16_t tid, uint32_t timeout_ms)
{
    uint8_t  chunk[GW_MB_RX_CHUNK];
    uint32_t deadline;

    m->got_frame = false;
    m->frame_len = 0u;

    /* 路径 A：外部帧来源（DMA + 环形缓冲 + MBAP 长度定界） */
    if (m->frame_source != NULL) {
        uint32_t len = 0u;
        gw_mbap_t hdr;
        const uint8_t *rpdu = NULL;
        uint32_t rpdu_len = 0u;
        int rc = m->frame_source(m->frame_source_ctx, m->frame,
                                 (uint32_t)sizeof(m->frame), &len, timeout_ms);
        if (rc != GW_OK) {
            m->timeouts++;
            return GW_ERR_TIMEOUT;
        }
        m->frame_len = len;
        m->got_frame = true;
        m->rx_frames++;
        if (gw_mb_tcp_parse(m->frame, len, &hdr, &rpdu, &rpdu_len) != GW_OK) {
            return GW_ERR_TIMEOUT;
        }
        if (hdr.transaction_id != tid) {
            m->tid_mismatch++;
            return GW_ERR_TIMEOUT;
        }
        return GW_OK;
    }

    gw_mb_tcp_asm_init(&m->asm_);
    deadline = gw_port_tick_ms() + timeout_ms;

    for (;;) {
        int32_t remain = (int32_t)(deadline - gw_port_tick_ms());
        int     n;

        if (remain <= 0) {
            m->timeouts++;
            return GW_ERR_TIMEOUT;
        }

        n = gw_link_recv(m->link, chunk, sizeof(chunk), remain);
        if (n > 0) {
            gw_mb_tcp_asm_feed(&m->asm_, chunk, (uint32_t)n, tcp_frame_cb, m);
            if (m->got_frame) {
                gw_mbap_t       hdr;
                const uint8_t  *rpdu = NULL;
                uint32_t        rpdu_len = 0u;

                if (gw_mb_tcp_parse(m->frame, m->frame_len, &hdr,
                                    &rpdu, &rpdu_len) != GW_OK) {
                    continue;
                }
                if (hdr.transaction_id != tid) {
                    /* 事务号不匹配：可能是上一个超时请求的迟到响应, 丢弃 */
                    m->tid_mismatch++;
                    m->got_frame = false;
                    continue;
                }
                return GW_OK;
            }
            continue;
        }
        if ((n == GW_ERR_TIMEOUT) || (n == GW_ERR_EMPTY)) {
            continue;
        }
        return GW_ERR_IO;
    }
}

int gw_mb_tcp_master_txn(gw_mb_tcp_master_t *m, uint8_t unit, const uint8_t *pdu,
                         uint32_t pdu_len, uint8_t *resp_adu, uint32_t cap,
                         uint32_t *resp_len)
{
    uint8_t  req[GW_MAX_ADU_TCP];
    uint32_t req_len = 0u;
    uint16_t tid;
    uint32_t attempt;
    int      last = GW_ERR_TIMEOUT;

    if ((m == NULL) || (pdu == NULL) || (resp_adu == NULL) || (resp_len == NULL)) {
        return GW_ERR_PARAM;
    }

    tid = m->next_tid++;
    if (m->next_tid == 0u) {
        m->next_tid = 1u;
    }

    if (gw_mb_tcp_build(tid, unit, pdu, pdu_len, req, sizeof(req), &req_len) != GW_OK) {
        return GW_ERR_PARAM;
    }

    for (attempt = 0u; attempt <= m->retries; attempt++) {
        gw_mbap_t       hdr;
        const uint8_t  *rpdu = NULL;
        uint32_t        rpdu_len = 0u;
        int             rc;
        int             n;

        if (attempt > 0u) {
            m->retry_count++;
            GW_LOGW("MBTCP", "retry %u/%u after %s", (unsigned)attempt,
                    (unsigned)m->retries, gw_status_str(last));
        }
        m->last_attempts = attempt + 1u;

        m->tx_frames++;
        n = gw_link_send(m->link, req, req_len, (int32_t)m->timeout_ms);
        if (n < 0) {
            m->last_error = GW_MB_ERR_LINK;
            return GW_ERR_IO;
        }

        rc = tcp_wait_frame(m, tid, m->timeout_ms);
        if (rc != GW_OK) {
            last = rc;
            m->last_error = GW_MB_ERR_TIMEOUT;
            continue;
        }

        rc = gw_mb_tcp_parse(m->frame, m->frame_len, &hdr, &rpdu, &rpdu_len);
        if (rc != GW_OK) {
            last = rc;
            m->last_error = GW_MB_ERR_BAD_LENGTH;
            continue;
        }
        if (hdr.unit_id != unit) {
            last = GW_ERR_NOT_FOUND;
            m->last_error = GW_MB_ERR_BAD_SLAVE;
            continue;
        }
        if (m->frame_len > cap) {
            return GW_ERR_OVERFLOW;
        }

        memcpy(resp_adu, m->frame, m->frame_len);
        *resp_len = m->frame_len;

        if (gw_mb_is_exception_function(rpdu[0])) {
            m->exceptions++;
            m->last_exception = (rpdu_len >= 2u) ? rpdu[1] : 0u;
            m->last_error     = GW_MB_ERR_EXCEPTION;
            return GW_ERR_EXCEPTION;       /* 明确答复, 不重试 */
        }
        m->ok_frames++;
        m->last_error = GW_MB_ERR_NONE;
        return GW_OK;
    }

    m->last_error = GW_MB_ERR_TIMEOUT;
    return last;
}

static int tcp_txn(gw_mb_tcp_master_t *m, uint8_t unit, const uint8_t *pdu,
                   uint32_t pdu_len, gw_mb_response_t *presp)
{
    uint8_t        adu[GW_MAX_ADU_TCP];
    uint32_t       adu_len = 0u;
    gw_mbap_t      hdr;
    const uint8_t *rpdu = NULL;
    uint32_t       rpdu_len = 0u;
    int            rc;

    rc = gw_mb_tcp_master_txn(m, unit, pdu, pdu_len, adu, sizeof(adu), &adu_len);
    if (rc != GW_OK) {
        return rc;
    }
    rc = gw_mb_tcp_parse(adu, adu_len, &hdr, &rpdu, &rpdu_len);
    if (rc != GW_OK) {
        return rc;
    }
    rc = gw_mb_pdu_parse_response(rpdu, rpdu_len, presp);
    presp->slave = unit;
    return rc;
}

int gw_mb_tcp_master_read(gw_mb_tcp_master_t *m, uint8_t unit, uint8_t fc,
                          uint16_t addr, uint16_t qty, uint16_t *out)
{
    uint8_t  pdu[8];
    uint32_t pdu_len = 0u;
    gw_mb_response_t pr;
    int      rc;
    uint16_t i;

    if ((m == NULL) || (out == NULL)) {
        return GW_ERR_PARAM;
    }
    rc = gw_mb_pdu_build_read(fc, addr, qty, pdu, sizeof(pdu), &pdu_len);
    if (rc != GW_OK) {
        return rc;
    }
    rc = tcp_txn(m, unit, pdu, pdu_len, &pr);
    if (rc != GW_OK) {
        return rc;
    }
    if (pr.value_count != qty) {
        return GW_ERR_STATE;
    }
    for (i = 0u; i < qty; i++) {
        out[i] = pr.values[i];
    }
    return GW_OK;
}

int gw_mb_tcp_master_write_single(gw_mb_tcp_master_t *m, uint8_t unit,
                                  uint16_t addr, uint16_t value)
{
    uint8_t  pdu[8];
    uint32_t pdu_len = 0u;
    gw_mb_response_t pr;
    int rc;

    if (m == NULL) {
        return GW_ERR_PARAM;
    }
    rc = gw_mb_pdu_build_write_single(addr, value, pdu, sizeof(pdu), &pdu_len);
    if (rc != GW_OK) {
        return rc;
    }
    rc = tcp_txn(m, unit, pdu, pdu_len, &pr);
    if (rc != GW_OK) {
        return rc;
    }
    return ((pr.start_addr == addr) && (pr.value == value)) ? GW_OK : GW_ERR_STATE;
}

int gw_mb_tcp_master_write_multi(gw_mb_tcp_master_t *m, uint8_t unit,
                                 uint16_t addr, const uint16_t *values,
                                 uint16_t qty)
{
    uint8_t  pdu[GW_MAX_ADU_TCP];
    uint32_t pdu_len = 0u;
    gw_mb_response_t pr;
    int rc;

    if ((m == NULL) || (values == NULL)) {
        return GW_ERR_PARAM;
    }
    rc = gw_mb_pdu_build_write_multi(addr, values, qty, pdu, sizeof(pdu), &pdu_len);
    if (rc != GW_OK) {
        return rc;
    }
    rc = tcp_txn(m, unit, pdu, pdu_len, &pr);
    if (rc != GW_OK) {
        return rc;
    }
    return ((pr.start_addr == addr) && (pr.quantity == qty)) ? GW_OK : GW_ERR_STATE;
}

/* ================================================================== */
/* 从站（进程内模拟）                                                  */
/* ================================================================== */
int gw_mb_tcp_slave_process(gw_mb_slave_t *s, const uint8_t *req, uint32_t req_len,
                            uint8_t *resp, uint32_t cap, uint32_t *resp_len)
{
    gw_mbap_t       hdr;
    const uint8_t  *pdu = NULL;
    uint32_t        pdu_len = 0u;
    gw_mb_request_t rq;
    uint8_t         rpdu[GW_MAX_ADU_TCP];
    uint32_t        rpdu_len = 0u;
    int             rc;

    if ((s == NULL) || (req == NULL) || (resp == NULL) || (resp_len == NULL)) {
        return GW_ERR_PARAM;
    }

    rc = gw_mb_tcp_parse(req, req_len, &hdr, &pdu, &pdu_len);
    if (rc != GW_OK) {
        return rc;
    }
    if (hdr.unit_id != s->addr) {
        return GW_ERR_NOT_FOUND;
    }

    s->requests_handled++;
    rc = gw_mb_pdu_parse_request(pdu, pdu_len, &rq);
    if (rc == GW_ERR_UNSUPPORTED) {
        s->exceptions_sent++;
        (void)gw_mb_pdu_build_exception(pdu[0], GW_MB_EXC_ILLEGAL_FUNCTION,
                                        rpdu, sizeof(rpdu), &rpdu_len);
        return gw_mb_tcp_build(hdr.transaction_id, s->addr, rpdu, rpdu_len,
                               resp, cap, resp_len);
    }
    if (rc != GW_OK) {
        s->exceptions_sent++;
        (void)gw_mb_pdu_build_exception(pdu[0], GW_MB_EXC_ILLEGAL_VALUE,
                                        rpdu, sizeof(rpdu), &rpdu_len);
        return gw_mb_tcp_build(hdr.transaction_id, s->addr, rpdu, rpdu_len,
                               resp, cap, resp_len);
    }

    switch (rq.function) {
    case GW_MB_FC_READ_HOLDING:
    case GW_MB_FC_READ_INPUT: {
        const uint16_t *tbl = (rq.function == GW_MB_FC_READ_HOLDING)
                            ? s->holding : s->input;
        uint16_t cnt = (rq.function == GW_MB_FC_READ_HOLDING)
                     ? s->holding_count : s->input_count;
        if (((uint32_t)rq.start_addr + rq.quantity) > cnt) {
            s->exceptions_sent++;
            (void)gw_mb_pdu_build_exception(rq.function, GW_MB_EXC_ILLEGAL_ADDRESS,
                                            rpdu, sizeof(rpdu), &rpdu_len);
            break;
        }
        rc = gw_mb_pdu_build_read_response(rq.function, &tbl[rq.start_addr],
                                           rq.quantity, rpdu, sizeof(rpdu),
                                           &rpdu_len);
        if (rc != GW_OK) {
            return rc;
        }
        break;
    }

    case GW_MB_FC_WRITE_SINGLE_REG:
        if (rq.start_addr >= s->holding_count) {
            s->exceptions_sent++;
            (void)gw_mb_pdu_build_exception(rq.function, GW_MB_EXC_ILLEGAL_ADDRESS,
                                            rpdu, sizeof(rpdu), &rpdu_len);
            break;
        }
        s->holding[rq.start_addr] = rq.value;
        (void)gw_mb_pdu_build_write_response(rq.function, rq.start_addr, rq.value,
                                             rpdu, sizeof(rpdu), &rpdu_len);
        break;

    case GW_MB_FC_WRITE_MULTI_REGS: {
        uint16_t i;
        if (((uint32_t)rq.start_addr + rq.quantity) > s->holding_count) {
            s->exceptions_sent++;
            (void)gw_mb_pdu_build_exception(rq.function, GW_MB_EXC_ILLEGAL_ADDRESS,
                                            rpdu, sizeof(rpdu), &rpdu_len);
            break;
        }
        for (i = 0u; i < rq.quantity; i++) {
            s->holding[rq.start_addr + i] = rq.values[i];
        }
        (void)gw_mb_pdu_build_write_response(rq.function, rq.start_addr,
                                             rq.quantity, rpdu, sizeof(rpdu),
                                             &rpdu_len);
        break;
    }

    default:
        s->exceptions_sent++;
        (void)gw_mb_pdu_build_exception(rq.function, GW_MB_EXC_ILLEGAL_FUNCTION,
                                        rpdu, sizeof(rpdu), &rpdu_len);
        break;
    }

    return gw_mb_tcp_build(hdr.transaction_id, s->addr, rpdu, rpdu_len,
                           resp, cap, resp_len);
}
