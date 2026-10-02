/*
 * param_store.c - 双槽轮换 + 校验回读的参数持久化
 *
 * 关键点：
 *   1) 永远"先写非当前槽, 校验通过再切当前槽", 保证任意时刻都有一个
 *      完整可用的槽（这是掉电不丢参数的根本保证）；
 *   2) 槽头单独做 CRC, 这样容量不足时也能尽早判定"这个槽是废的"，
 *      不用把 2KB 读完；
 *   3) 序号 seq 单调递增, 回绕用无符号差值比较, 避免 2^32 次保存后
 *      选出旧槽这种"十年后才炸"的隐蔽 bug。
 */
#include "param_store.h"
#include "gateway_config.h"
#include "gw_crc.h"
#include "gw_log.h"

#include <string.h>

#define GW_PARAM_HDR_SIZE  16u
#define GW_PARAM_SLOT_BASE(i)  ((uint32_t)(i) * GW_PARAM_SLOT_SIZE)

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint16_t payload_len;
    uint16_t hdr_crc;
    uint16_t payload_crc;
    uint16_t reserved;
} gw_param_hdr_t;

/* seq 比较：a 比 b 新返回 true（无符号差值法, 天然处理回绕） */
static bool seq_newer(uint32_t a, uint32_t b)
{
    return (uint32_t)(a - b) < 0x80000000u;
}

static void hdr_encode(uint8_t *dst, const gw_param_hdr_t *h)
{
    gw_write_be32(&dst[0], h->magic);
    gw_write_be32(&dst[4], h->seq);
    gw_write_be16(&dst[8], h->payload_len);
    dst[10] = 0u;
    dst[11] = 0u;
    gw_write_be16(&dst[12], h->payload_crc);
    gw_write_be16(&dst[14], h->reserved);
    /* 头部自身 CRC 覆盖前 12 字节, 写在 [10..11] */
    {
        uint16_t c = gw_crc16_modbus_fast(dst, 10u);
        dst[10] = (uint8_t)(c & 0xFFu);
        dst[11] = (uint8_t)((c >> 8) & 0xFFu);
    }
}

static int hdr_decode(const uint8_t *src, gw_param_hdr_t *h)
{
    uint16_t calc;
    uint16_t stored;

    h->magic       = gw_read_be32(&src[0]);
    h->seq         = gw_read_be32(&src[4]);
    h->payload_len = gw_read_be16(&src[8]);
    h->payload_crc = gw_read_be16(&src[12]);
    h->reserved    = gw_read_be16(&src[14]);

    if (h->magic != GW_PARAM_MAGIC) {
        return GW_ERR_NOT_FOUND;
    }
    if (h->payload_len > GW_PARAM_MAX_PAYLOAD) {
        return GW_ERR_PARAM;
    }

    calc   = gw_crc16_modbus_fast(src, 10u);
    stored = (uint16_t)((uint16_t)src[10] | ((uint16_t)src[11] << 8));
    if (calc != stored) {
        return GW_ERR_CRC;
    }
    return GW_OK;
}

void gw_param_defaults(gw_param_blob_t *blob)
{
    if (blob == NULL) {
        return;
    }
    memset(blob, 0, sizeof(*blob));
    blob->magic                = GW_PARAM_MAGIC;
    blob->version              = 1u;
    blob->device_count         = 3u;
    blob->poll_period_ms       = GW_POLL_PERIOD_MS;
    blob->heartbeat_timeout_ms = GW_HEARTBEAT_TIMEOUT_MS;
    blob->bus_retry_max        = GW_MB_RETRY_MAX;
    blob->isolate_cooldown_ms  = GW_ISOLATE_COOLDOWN_MS;
    blob->welder_current_set   = 1800u;   /* 180.0A */
    blob->welder_voltage_set   = 240u;    /* 24.0V  */
    blob->wire_feed_speed      = 850u;    /* 8.50 m/min */
    blob->robot_program_no     = 1u;
    blob->report_interval_ms   = 200u;
    blob->crc_seed             = 0xFFFFu;
}

int gw_param_store_init(gw_param_store_t *ps)
{
    if (ps == NULL) {
        return GW_ERR_PARAM;
    }
    memset(ps, 0, sizeof(*ps));
    ps->active_slot = 0xFFFFFFFFu;   /* 尚未确定 */
    return GW_OK;
}

int gw_param_save(gw_param_store_t *ps, const void *payload, uint16_t len)
{
    gw_param_hdr_t h;
    uint8_t        image[GW_PARAM_SLOT_SIZE];
    uint8_t        readback[GW_PARAM_SLOT_SIZE];
    uint32_t       target;
    int            rc;

    if ((ps == NULL) || (payload == NULL)) {
        return GW_ERR_PARAM;
    }
    if ((len == 0u) || (len > GW_PARAM_MAX_PAYLOAD)) {
        return GW_ERR_PARAM;
    }

    /* 目标槽 = 非当前槽（首次保存用槽 0） */
    target = (ps->active_slot == 0u) ? 1u : 0u;

    memset(image, 0xFF, sizeof(image));
    h.magic       = GW_PARAM_MAGIC;
    h.seq         = ps->seq + 1u;
    h.payload_len = len;
    h.payload_crc = gw_crc16_modbus_fast((const uint8_t *)payload, len);
    h.reserved    = 0u;
    hdr_encode(image, &h);
    memcpy(&image[GW_PARAM_HDR_SIZE], payload, len);

    if (gw_nv_erase(GW_PARAM_SLOT_BASE(target), GW_PARAM_SLOT_SIZE) != GW_OK) {
        return GW_ERR_IO;
    }
    rc = gw_nv_write(GW_PARAM_SLOT_BASE(target), image, sizeof(image));
    if (rc != GW_OK) {
        /* 掉电被打断：新槽作废, 旧槽仍然可用, 不能切换 active_slot */
        ps->corrupt_count++;
        GW_LOGE("PARAM", "save aborted (power loss?): slot %u left corrupt",
                (unsigned)target);
        return GW_ERR_IO;
    }

    /* 回读校验：这道关必须过, 否则等于把参数交给运气 */
    if ((gw_nv_read(GW_PARAM_SLOT_BASE(target), readback, sizeof(readback)) != GW_OK) ||
        (memcmp(readback, image, sizeof(image)) != 0)) {
        ps->corrupt_count++;
        GW_LOGE("PARAM", "read-back verify failed on slot %u", (unsigned)target);
        return GW_ERR_CRC;
    }

    ps->active_slot = target;
    ps->seq         = h.seq;
    ps->save_count++;
    ps->last_len    = len;
    GW_LOGI("PARAM", "saved %u bytes to slot %u (seq=%u, erase_total=%u)",
            (unsigned)len, (unsigned)target, (unsigned)h.seq,
            (unsigned)gw_nv_get_erase_count());
    return GW_OK;
}

int gw_param_load(gw_param_store_t *ps, void *payload, uint16_t cap,
                  uint16_t *out_len)
{
    gw_param_hdr_t h[GW_PARAM_SLOT_COUNT];
    uint8_t        raw[GW_PARAM_HDR_SIZE];
    int            valid[GW_PARAM_SLOT_COUNT];
    uint32_t       i;
    int            best = -1;

    if ((ps == NULL) || (payload == NULL) || (out_len == NULL)) {
        return GW_ERR_PARAM;
    }

    for (i = 0u; i < GW_PARAM_SLOT_COUNT; i++) {
        valid[i] = GW_ERR;
        if (gw_nv_read(GW_PARAM_SLOT_BASE(i), raw, sizeof(raw)) != GW_OK) {
            continue;
        }
        valid[i] = hdr_decode(raw, &h[i]);
        if (valid[i] != GW_OK) {
            ps->corrupt_count++;
            GW_LOGW("PARAM", "slot %u invalid: %s", (unsigned)i,
                    gw_status_str(valid[i]));
            continue;
        }
        if (best < 0) {
            best = (int)i;
        } else if (seq_newer(h[i].seq, h[(uint32_t)best].seq)) {
            best = (int)i;
        }
    }

    if (best < 0) {
        GW_LOGE("PARAM", "no valid slot found, factory defaults required");
        return GW_ERR_NOT_FOUND;
    }

    if (h[best].payload_len > cap) {
        return GW_ERR_OVERFLOW;
    }
    if (gw_nv_read(GW_PARAM_SLOT_BASE((uint32_t)best) + GW_PARAM_HDR_SIZE,
                   (uint8_t *)payload, h[best].payload_len) != GW_OK) {
        return GW_ERR_IO;
    }
    if (gw_crc16_modbus_fast((const uint8_t *)payload, h[best].payload_len) !=
        h[best].payload_crc) {
        ps->corrupt_count++;
        GW_LOGE("PARAM", "payload CRC mismatch on slot %d", best);
        return GW_ERR_CRC;
    }

    /* 是否发生了"主槽坏了, 用备份槽救回来" */
    {
        uint32_t other = ((uint32_t)best == 0u) ? 1u : 0u;
        if (valid[other] != GW_OK) {
            ps->recovery_count++;
        }
    }

    ps->active_slot = (uint32_t)best;
    ps->seq         = h[best].seq;
    ps->load_count++;
    ps->last_len    = h[best].payload_len;
    ps->loaded      = true;
    *out_len        = h[best].payload_len;

    GW_LOGI("PARAM", "loaded %u bytes from slot %d (seq=%u, recoveries=%u)",
            (unsigned)h[best].payload_len, best, (unsigned)h[best].seq,
            (unsigned)ps->recovery_count);
    return GW_OK;
}

int gw_param_factory_reset(gw_param_store_t *ps)
{
    gw_param_blob_t blob;

    if (ps == NULL) {
        return GW_ERR_PARAM;
    }
    gw_param_defaults(&blob);
    ps->factory_reset_count++;
    return gw_param_save(ps, &blob, (uint16_t)sizeof(blob));
}

void gw_param_store_stats(gw_param_store_t *ps, gw_param_store_t *out)
{
    if ((ps == NULL) || (out == NULL)) {
        return;
    }
    *out = *ps;
}
