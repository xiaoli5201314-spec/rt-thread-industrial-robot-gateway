/*
 * frame_sync.c - 帧同步状态机实现
 *
 * 状态迁移：
 *
 *   IDLE ──收到第 1 个字节──▶ RECEIVING
 *   RECEIVING ──CRC 通过──▶ IDLE（上报 FRAME）
 *   RECEIVING ──静默 > T3.5 且不完整──▶ IDLE（上报 SHORT_FRAME）
 *   RECEIVING ──长度达到期望但 CRC 错──▶ RESYNC（上报 CRC_ERROR）
 *   RESYNC ──逐字节滑动直到 CRC 通过或缓冲耗尽──▶ RECEIVING / IDLE
 *
 * 之所以把"CRC 通过"放在静默判据之前：上位机 USB 转串口驱动会把
 * 静默间隔打散, 只依赖定时器判帧会大面积丢帧；CRC 判据在字节流
 * 被任意切分的情况下仍然稳定, 静默判据只作为兜底与错误分类依据。
 */
#include "frame_sync.h"
#include "gw_crc.h"
#include "gw_log.h"

#include <string.h>

const char *gw_fs_event_str(gw_fs_event_kind_t k)
{
    switch (k) {
    case GW_FS_EV_NONE:        return "NONE";
    case GW_FS_EV_FRAME:       return "FRAME";
    case GW_FS_EV_CRC_ERROR:   return "CRC_ERROR";
    case GW_FS_EV_SHORT_FRAME: return "SHORT_FRAME";
    case GW_FS_EV_OVERSIZE:    return "OVERSIZE";
    case GW_FS_EV_RESYNC:      return "RESYNC";
    default:                   return "UNKNOWN";
    }
}

int gw_frame_sync_init(gw_frame_sync_t *fs, uint8_t *buf, uint32_t cap,
                       uint32_t t35_us, gw_fs_len_fn len_fn,
                       gw_fs_event_cb cb, void *cb_ctx)
{
    if ((fs == NULL) || (buf == NULL) || (cap < 8u)) {
        return GW_ERR_PARAM;
    }
    memset(fs, 0, sizeof(*fs));
    fs->buf      = buf;
    fs->cap      = cap;
    fs->t35_us   = (t35_us == 0u) ? GW_T35_US : t35_us;
    fs->len_fn   = len_fn;
    fs->cb       = cb;
    fs->cb_ctx   = cb_ctx;
    fs->state    = GW_FS_IDLE;
    return GW_OK;
}

void gw_frame_sync_reset(gw_frame_sync_t *fs)
{
    if (fs == NULL) {
        return;
    }
    fs->len           = 0u;
    fs->state         = GW_FS_IDLE;
    fs->last_byte_us  = 0u;
    fs->frame_start_us = 0u;
}

/* 默认完整性校验：CRC16/Modbus */
static bool fs_crc_default(const uint8_t *buf, uint32_t len)
{
    return gw_crc16_modbus_check_frame(buf, len);
}

/* 对缓冲前 n 字节做完整性校验；crc_fn 为 NULL 时使用 CRC16/Modbus */
static bool fs_crc_ok(gw_frame_sync_t *fs, uint32_t n)
{
    if (fs->crc_fn != NULL) {
        return fs->crc_fn(fs->buf, n);
    }
    return fs_crc_default(fs->buf, n);
}

void gw_frame_sync_set_crc_fn(gw_frame_sync_t *fs, gw_fs_crc_fn crc_fn)
{
    if (fs != NULL) {
        fs->crc_fn = crc_fn;
    }
}

static void fs_emit(gw_frame_sync_t *fs, gw_fs_event_kind_t kind,
                    uint32_t len, uint32_t ts, uint32_t dropped)
{
    if (fs->cb != NULL) {
        gw_fs_event_t ev;
        ev.kind         = kind;
        ev.data         = fs->buf;
        ev.len          = len;
        ev.timestamp_us = ts;
        ev.dropped      = dropped;
        fs->cb(fs->cb_ctx, &ev);
    }
}

/* 丢弃缓冲头部 n 字节, 把剩余数据前移 */
static void fs_drop_head(gw_frame_sync_t *fs, uint32_t n)
{
    if (n >= fs->len) {
        fs->len = 0u;
        return;
    }
    memmove(fs->buf, fs->buf + n, fs->len - n);
    fs->len -= n;
}

/* 期望长度：优先用协议推断, 推断不出时返回 0。
 * 关键约束：返回值只用于"帧是否收全"的判断与错误分类, **不**用于
 * 决定何时滑动重同步 —— 缓冲头部混入噪声时推断出的长度可能很大,
 * 一旦拿它当滑动门限, 噪声就会长期占住缓冲, 后面的正确帧永远收不到。 */
static uint32_t fs_expected_len(gw_frame_sync_t *fs)
{
    if ((fs->len_fn == NULL) || (fs->len < 2u)) {
        return 0u;
    }
    return fs->len_fn(fs->buf, fs->len);
}

/*
 * 头部可行性：功能码非法说明缓冲头这一字节肯定是噪声, 立刻丢掉。
 * 这是"错帧后快速重新对齐"的关键 —— 逐字节滑动时绝大多数窗口的
 * 第 2 个字节都不是合法功能码, 一步即可排除, 不必等到收满整帧长度。
 */
static bool fs_header_plausible(const gw_frame_sync_t *fs)
{
    uint8_t fc;

    if (fs->len < 2u) {
        return true;
    }
    fc = (uint8_t)(fs->buf[1] & 0x7Fu);
    switch (fc) {
    case 0x01u: case 0x02u: case 0x03u: case 0x04u:
    case 0x05u: case 0x06u: case 0x0Fu: case 0x10u:
        return true;
    default:
        return false;
    }
}

/* 丢弃缓冲头 1 字节, 进入重同步状态（滑动对齐） */
static void fs_slide_one(gw_frame_sync_t *fs)
{
    fs->state = GW_FS_RESYNC;
    fs->resync_count++;
    fs->garbage_bytes++;
    fs_drop_head(fs, 1u);
}

/*
 * 滑动重同步：在已累积的字节里寻找下一个"CRC 正确"的候选帧。
 * 返回 true 表示找到并已上报了一帧；返回 false 表示已尽力对齐,
 * 剩余字节还不足以判定, 保留下来等待更多数据。
 */
static bool fs_resync_scan(gw_frame_sync_t *fs, uint32_t ts)
{
    uint32_t dropped = 0u;

    for (;;) {
        /* 1) 头部功能码非法 -> 该字节是噪声, 立刻滑动（不产生事件风暴） */
        if (!fs_header_plausible(fs)) {
            fs_slide_one(fs);
            dropped++;
            continue;
        }

        if (fs->len < 4u) {
            break;                      /* 还不足以判定 */
        }

        /* 2) 整段恰好构成一帧（含"重同步后正好对齐"的场景） */
        if (fs_crc_ok(fs, fs->len)) {
            fs->frames_ok++;
            fs_emit(fs, GW_FS_EV_FRAME, fs->len, ts, dropped);
            fs->len   = 0u;
            fs->state = GW_FS_IDLE;
            return true;
        }

        /* 3) 长度已够而 CRC 不通过 -> 真正的错帧, 滑动 1 字节重新对齐 */
        {
            uint32_t expect = fs_expected_len(fs);
            if ((expect > 0u) && (fs->len >= expect)) {
                if (fs_crc_ok(fs, expect)) {
                    fs->frames_ok++;
                    fs_emit(fs, GW_FS_EV_FRAME, expect, ts, dropped);
                    fs_drop_head(fs, expect);
                    continue;
                }
                fs->crc_errors++;
                fs_emit(fs, GW_FS_EV_CRC_ERROR, expect, ts, dropped);
                fs_slide_one(fs);
                dropped = 0u;
                continue;
            }
        }

        if (fs->len >= fs->cap) {
            fs->oversize++;
            fs_emit(fs, GW_FS_EV_OVERSIZE, fs->len, ts, dropped);
            fs_slide_one(fs);
            dropped = 0u;
            continue;
        }
        break;   /* 数据还不够, 等更多字节 */
    }

    if (fs->len == 0u) {
        fs->state = GW_FS_IDLE;
    } else if (fs->state == GW_FS_RESYNC) {
        fs->state = GW_FS_RECEIVING;
    }
    return false;
}

/* 静默到达：当前缓冲要么是一帧, 要么是残帧 */
static void fs_close_on_silence(gw_frame_sync_t *fs, uint32_t ts)
{
    if (fs->len == 0u) {
        fs->state = GW_FS_IDLE;
        return;
    }

    if (fs_crc_ok(fs, fs->len)) {
        fs->frames_ok++;
        fs_emit(fs, GW_FS_EV_FRAME, fs->len, ts, 0u);
    } else if (fs->len < 4u) {
        /* 太少, 不可能构成一帧：按噪声统计 */
        fs->short_frames++;
        fs->garbage_bytes += fs->len;
        fs_emit(fs, GW_FS_EV_SHORT_FRAME, fs->len, ts, 0u);
    } else {
        uint32_t expect = fs_expected_len(fs);
        if ((expect > 0u) && (fs->len < expect)) {
            /* 帧没传完就静默了 —— 典型"丢帧/掉线", 与 CRC 错区分开,
             * 因为两者的现场处置方式不同（前者查链路, 后者查干扰） */
            fs->short_frames++;
            fs_emit(fs, GW_FS_EV_SHORT_FRAME, fs->len, ts, 0u);
        } else {
            fs->crc_errors++;
            fs_emit(fs, GW_FS_EV_CRC_ERROR, fs->len, ts, 0u);
        }
    }

    fs->len   = 0u;
    fs->state = GW_FS_IDLE;
}

void gw_frame_sync_feed(gw_frame_sync_t *fs, const uint8_t *data, uint32_t len,
                        uint32_t timestamp_us)
{
    uint32_t i;

    if ((fs == NULL) || (data == NULL) || (len == 0u)) {
        return;
    }

    for (i = 0u; i < len; i++) {
        uint8_t byte = data[i];

        /* 规则 2：静默间隔先结算上一帧（分包 / 丢帧场景） */
        if ((fs->state != GW_FS_IDLE) && (fs->len > 0u) &&
            ((timestamp_us - fs->last_byte_us) > fs->t35_us)) {
            fs_close_on_silence(fs, timestamp_us);
        }

        if (fs->state == GW_FS_IDLE) {
            fs->frame_start_us = timestamp_us;
            fs->state          = GW_FS_RECEIVING;
        }

        if (fs->len >= fs->cap) {
            /* 兜底：缓冲塞满仍未判出帧 */
            fs->oversize++;
            fs_emit(fs, GW_FS_EV_OVERSIZE, fs->len, timestamp_us, 0u);
            fs_slide_one(fs);
        }

        fs->buf[fs->len++] = byte;
        fs->last_byte_us   = timestamp_us;
        fs->bytes_in++;

        /* 规则 1：CRC 优先判帧 —— 这就是"粘包"能被切开的原因 */
        if ((fs->len >= 4u) && fs_crc_ok(fs, fs->len)) {
            fs->frames_ok++;
            fs_emit(fs, GW_FS_EV_FRAME, fs->len, timestamp_us, 0u);
            fs->len   = 0u;
            fs->state = GW_FS_IDLE;
            continue;
        }

        /* 规则 3/4：错帧与滑动重同步（头部不可行也会在这里被丢弃） */
        (void)fs_resync_scan(fs, timestamp_us);
    }
}

void gw_frame_sync_tick(gw_frame_sync_t *fs, uint32_t timestamp_us)
{
    if (fs == NULL) {
        return;
    }
    if ((fs->len > 0u) && ((timestamp_us - fs->last_byte_us) > fs->t35_us)) {
        fs_close_on_silence(fs, timestamp_us);
    }
}

void gw_frame_sync_get_stats(const gw_frame_sync_t *fs, gw_frame_sync_stats_t *out)
{
    if ((fs == NULL) || (out == NULL)) {
        return;
    }
    out->frames_ok     = fs->frames_ok;
    out->crc_errors    = fs->crc_errors;
    out->short_frames  = fs->short_frames;
    out->oversize      = fs->oversize;
    out->resync_count  = fs->resync_count;
    out->bytes_in      = fs->bytes_in;
    out->garbage_bytes = fs->garbage_bytes;
}

gw_fs_state_t gw_frame_sync_state(const gw_frame_sync_t *fs)
{
    return (fs != NULL) ? fs->state : GW_FS_IDLE;
}
